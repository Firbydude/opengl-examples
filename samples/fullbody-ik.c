/* Copyright (c) 2015 Scott Kuhl. All rights reserved.
 * License: This code is licensed under a 3-clause BSD license. See
 * the file named "LICENSE" for a full copy of the license.
 */

/** @file Demonstrates inverse kinematics using a Jacobian transpose approach.
 *
 * cmake -DCMAKE_LIBRARY_PATH="../vrpn;../vrpn/quat" .
 *
 * @author Scott Kuhl
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "inverse_kinematics.h"

#include "libkuhl.h"

static GLuint program = 0; /**< id value for the GLSL program */

static int renderStyle = 0;

static kuhl_geometry *modelgeom = NULL;
static float bbox[6];

/** Set this variable to 1 to force this program to scale the entire
 * model and translate it so that we can see the entire model. This is
 * a useful setting to use when you are loading a new model that you
 * are unsure about the units and position of the model geometry. */
#define FIT_TO_VIEW 0
/** If FIT_TO_VIEW is set, this is the place to put the
 * center of the bottom face of the bounding box. If
 * FIT_TO_VIEW is not set, this is the location in world
 * coordinates that we want to model's origin to appear at. */
static float placeToPutModel[3] = { 0, 0, 0 };

#define GLSL_VERT_FILE "assimp.vert"
#define GLSL_FRAG_FILE "assimp.frag"

static float angles[] = {
	10, 15, 20,  // arm 1
    0, 270, 30,  // arm 2
};
static int anglesCount = 6;
static float target[4] = { 0, 4, 0, 1};

#define THICKNESS 0.04

/**
 * Maximum number of jacobian inverse operations for each frame. Even using the
 * naive transpose method, if its over 250 then we're likely trying to reach an
 * impossible position.
 */
#define MAX_INVERSE_ITERATIONS 250

static struct kuhl_ik *ik;

/***** Tracking *****/

#define USE_VRPN 0
#define VRPN_HOST "localhost"
#define TRACKING_HANDR "Hand5R"
#define TRACKING_HANDL "Hand5L"
#define TRACKING_FOOTL "Foot5L"
#define TRACKING_FOOTR "Foot5R"

struct tracking_object {
	char *tracking_name;
	struct kuhl_skeleton *effector;
};

struct tracking_object effectors[4];

int active_effector = 0;

/***** Joint constraint stuff. *****/

#define UNCONSTRAINED 500
static float constraints[] = {
	// Arm 1
	UNCONSTRAINED,	UNCONSTRAINED, // X axis
	UNCONSTRAINED,	UNCONSTRAINED, // Y axis
	UNCONSTRAINED,	UNCONSTRAINED, // Z axis

	// Arm 2
	0,				0,		// X axis
	180,			360,	// Y axis
	0,				135		// Z axis
};

int is_constrained(int angle)
{
	return constraints[angle * 2] != UNCONSTRAINED;
}

float cmin(int angle)
{
	return constraints[angle * 2];
}

float cmax(int angle )
{
	return constraints[angle * 2 + 1];
}

/*******************************************/

void update_effector_target(float x, float y, float z)
{
	float target[3];
	ik_get_effector_target(ik, effectors[active_effector].effector, target);
	target[0] += x;
	target[1] += y;
	target[2] += z;
	ik_set_effector_target(ik, effectors[active_effector].effector, target);
}

/* Called by GLFW whenever a key is pressed. */
void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if(action != GLFW_PRESS && action != GLFW_REPEAT)
		return;

	switch(key)
	{
		case GLFW_KEY_Q:
		case GLFW_KEY_ESCAPE:
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
#if 0
		case 'f': // full screen
			glutFullScreen();
			break;
		case 'F': // switch to window from full screen mode
			glutPositionWindow(0,0);
			break;
#endif
		case GLFW_KEY_A: update_effector_target(.02, 0, 0); break;
		case GLFW_KEY_D: update_effector_target(-.02, 0, 0); break;
		case GLFW_KEY_W: update_effector_target(0, .02, 0); break;
		case GLFW_KEY_S: update_effector_target(0, -.02, 0); break;
		case GLFW_KEY_X: update_effector_target(0, 0, .02); break;
		case GLFW_KEY_Z: update_effector_target(0, 0, -.02); break;
		case GLFW_KEY_R:
		{
			// Reload GLSL program from disk
			kuhl_delete_program(program);
			program = kuhl_create_program(GLSL_VERT_FILE, GLSL_FRAG_FILE);
			/* Apply the program to the model geometry */
			kuhl_geometry_program(modelgeom, program, KG_FULL_LIST);
			break;
		}
		case GLFW_KEY_1:
		case GLFW_KEY_2:
		case GLFW_KEY_3:
		case GLFW_KEY_4:
			active_effector = key - GLFW_KEY_1;
			break;
	}
}


/** Gets a model matrix which is appropriate for the model that we have loaded. */
void get_model_matrix(float result[16])
{
	mat4f_identity(result);
	if(FIT_TO_VIEW == 0)
	{
		/* Translate the model to where we were asked to put it */
		float translate[16];
		mat4f_translateVec_new(translate, placeToPutModel);

		/* Do inches to meters conversion if we are asked to. */
		float scale[16];
		mat4f_identity(scale);
		mat4f_mult_mat4f_new(result, translate, scale);
		return;
	}

	/* Get a matrix to scale+translate the model based on the bounding
	 * box. If the last parameter is 1, the bounding box will sit on
	 * the XZ plane. If it is set to 0, the bounding box will be
	 * centered at the specified point. */
	float fitMatrix[16];
	kuhl_bbox_fit(fitMatrix, bbox, 1);

	/* Get a matrix that moves the model to the correct location. */
	float moveToLookPoint[16];
	mat4f_translateVec_new(moveToLookPoint, placeToPutModel);

	/* Create a single model matrix. */
	mat4f_mult_mat4f_new(result, moveToLookPoint, fitMatrix);
}


/** Get arm matrices given a set of angles. The arm2 matrix already has
 * the arm1 matrix applied to it. */
void get_arm_matrices(float arm1[16], float arm2[16], float angles[])
{
	list *stack = list_new(16, sizeof(float)*16, NULL);

	float baseRotate[16];
	mat4f_rotateEuler_new(baseRotate, angles[0], angles[1], angles[2], "XYZ");
	mat4f_stack_mult(stack, baseRotate);
	mat4f_stack_push(stack);

	float scale[16];
	mat4f_scale_new(scale, .5, 4, .5);
	float decenter[16];
	mat4f_translate_new(decenter, 0, .5, 0);

	mat4f_stack_mult(stack, scale);
	mat4f_stack_mult(stack, decenter);
	mat4f_stack_peek(stack, arm1);
	mat4f_stack_pop(stack);

	float trans[16];
	mat4f_translate_new(trans, 0, 4, 0);
	mat4f_stack_mult(stack, trans);

	mat4f_rotateEuler_new(baseRotate, angles[3], angles[4], angles[5], "XYZ");
	mat4f_stack_mult(stack, baseRotate);
	mat4f_stack_push(stack);

	mat4f_stack_mult(stack, scale);
	mat4f_stack_mult(stack, decenter);
	mat4f_stack_peek(stack, arm2);
	mat4f_stack_pop(stack);

	list_free(stack);
}

/* Given a list of angles, calculate end effector location */
void end_effector_loc(float loc[4], float angles[])
{
	float arm1mat[16], arm2mat[16];
	get_arm_matrices(arm1mat, arm2mat, angles);
	vec4f_set(loc, 0, .5, 0, 1);
	mat4f_mult_vec4f_new(loc, arm2mat, loc);
}

/* Get a jacobian matrix. It is 3 elements tall and angleCount
 * elements wide. Each column represents how (x,y,z) of the end
 * effector will change given a small change in the angle. */
float* get_jacobian(float delta)
{
	float *jacobian = malloc(sizeof(float)*3*anglesCount);

	float origLoc[4];
	end_effector_loc(origLoc, angles);

	for(int i=0; i<anglesCount; i++)
	{
		// Prevent movement on angles with no allowed movement.
		// This might not actually do what we want. Once we get test cases
		// set up we'll verify. It seems to reduce iterations at a glance.
		if (is_constrained(i) && cmin(i) == cmax(i))
		{
			for (int j = 0; j < 3; j++)
				jacobian[i * 3 + j] = 0;
		}
		else
		{
			angles[i] += delta;
			float newLoc[4];
			end_effector_loc(newLoc, angles);
			float deltaLoc[3];
			vec3f_sub_new(deltaLoc, newLoc, origLoc);
			for(int j=0; j<3; j++)
				jacobian[i*3+j] = deltaLoc[j];
			angles[i] -= delta;
		}
	}

	printf("jacobian:\n");
	for(int i=0; i<anglesCount; i++)
	{
		for(int j=0; j<3; j++)
			printf("%8.4f ", jacobian[i*3+j]);
		printf("\n");
	}

	return jacobian;
}


void effector_target(float target[4])
{
	int timesThroughLoop = 0;

	while(1)
	{
		/* Get current location of end effector */
		float currentLoc[4];
		end_effector_loc(currentLoc, angles);
		/* Get a vector pointing to target from current end effector location */
		float deltaTarget[3];
		vec3f_sub_new(deltaTarget, target, currentLoc);
		float distance = vec3f_norm(deltaTarget);

		timesThroughLoop++;
		if(distance < .001 || timesThroughLoop >= MAX_INVERSE_ITERATIONS)
		{
			if (timesThroughLoop > 1)
				printf("Times through loop: %d\n", timesThroughLoop);
			break;
		}

		printf("pre: location, target, delta:\n");
		vec3f_print(currentLoc);
		vec3f_print(target);
		vec3f_print(deltaTarget);
		printf("distance: %f\n", distance);
		printf("angles:\n");
		vecNf_print(angles, anglesCount);

		float *jacobian = get_jacobian(2);

		/* Jacobian is in column-major order. Each column represents
		 * the change in (x,y,z) given a change in a specific angle:

		   angle0  angle1  ...
		   -----------------
		   x       x
		   y       y
		   z       z

		   We can "transpose" the jacobian simply by changing the way
		   we index into the array. To transpose it, we simply assume
		   that the array is in row-major order---meaning that a row
		   represents a change in (x,y,z) given a change in a specific
		   angle.

		   Transposed jacobian is (we can assume it is in row-major order to transpose)

		   x y z
		   x y z
		   ...

		*/


		/* Multiply the change in the end effector by the transposed
		 * Jacobian. The result will approximate the change in angle
		 * of each of the joints that we want. */
		float *changeInAngle = malloc(sizeof(float)*anglesCount);
		for(int i=0; i<anglesCount; i++) // for each row in the transposed Jacobian
		{
			// take the dot product of the transposed jacobian row with the target position:
			changeInAngle[i] = vec3f_dot(&(jacobian[i*3]), deltaTarget);
		}

		/* Calculate how these changes in angle would influence end
		   effector based on the original Jacobian */
		float expectedChangeInEffector[3] = { 0,0,0 };
		for(int i=0; i<3; i++)
		{
			// Dot product of the first row of the jacobian with the changeInAngle
			for(int j=0; j<anglesCount; j++)
				expectedChangeInEffector[i] += changeInAngle[j] * jacobian[j*3+i];
		}
		printf("expected change in effector:\n");
		vec3f_print(expectedChangeInEffector);

		/* Calculate a reasonable alpha according to:
		   http://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
		*/
		float alpha = vec3f_dot(expectedChangeInEffector, deltaTarget) /
		              vec3f_dot(expectedChangeInEffector, expectedChangeInEffector);
		printf("alpha: %f\n", alpha);

		/* Apply our angle changes (multiplied by alpha) to the robots angles */
		printf("Change in angles: ");
		for(int i=0; i<anglesCount; i++)
		{
			angles[i] += alpha*changeInAngle[i];

			// Clamp constrained angles
			if (is_constrained(i))
			{
				if (angles[i] < cmin(i))
				{
					printf("Clamping angle %d (%.2f) to min (%.2f)\n",
						   i, angles[i], cmin(i));
					angles[i] = cmin(i);
				}
				else if (angles[i] > cmax(i))
				{
					printf("Clamping angle %d (%.2f) to max (%.2f)\n",
						   i, angles[i], cmax(i));
					angles[i] = cmax(i);
				}
			}

			angles[i] = fmod(angles[i], 360); // Keep angles within 0 to 360
			printf("%f ", changeInAngle[i]);
		}
		printf("\n");

		float newLoc[4];
		end_effector_loc(newLoc, angles);
		float actualChange[3];
		vec3f_sub_new(actualChange, newLoc, currentLoc);
		printf("Actual change in end effector\n");
		vec3f_print(actualChange);

		free(changeInAngle);
		free(jacobian);

		// Uncomment to see IK solution change
		// break;
	}

}

void init_model()
{
	struct kuhl_skeleton *sk;
	struct kuhl_skeleton *parent;
	struct kuhl_skeleton *attach;
	float decenter[16];
	int num_effectors = 0;

	ik = ik_alloc();

	// Head (skeleton root)
	sk = sk_alloc("Head");
	sk->is_static = 1;
	mat4f_scale_new(sk->scale_mat, 0.1, 0.1, 0.1);
	mat4f_translate_new(sk->trans_mat, 0, -0.11, 0);
	//mat4f_identity(sk->transform_matrix);
	mat4f_translate_new(sk->test_mat, 0, -0.5, 0);
	ik->sk = sk;

	// Invisible neck
	sk = attach = sk_alloc("Neck");
	mat4f_scale_new(sk->scale_mat, 0, 0, 0);
	mat4f_translate_new(sk->trans_mat, 0, -0.01, 0);
	mat4f_translate_new(sk->test_mat, 0, -0.5, 0);
	sk_add_child(ik->sk, sk);
	parent = sk;

	mat4f_translate_new(decenter, -0.5, 0, 0);

	// Left shoulder
	sk = sk_alloc("ShoulderL");
	sk->is_static = 1;
	mat4f_scale_new(sk->scale_mat, 0.2, THICKNESS, THICKNESS);
	mat4f_translate_new(sk->trans_mat, -0.19, 0, 0);
	// mat4f_mult_mat4f_new(sk->transform_matrix, trans_mat, scale_mat);
	mat4f_copy(sk->test_mat, decenter);
	sk_add_child(attach, sk);
	parent = sk;

	// Left Arm
	sk = sk_alloc("ArmL");
	mat4f_scale_new(sk->scale_mat, 0.2, THICKNESS, THICKNESS);
	mat4f_translate_new(sk->trans_mat, -0.21, 0, 0);
	// mat4f_mult_mat4f_new(sk->transform_matrix, trans_mat, scale_mat);
	mat4f_copy(sk->test_mat, decenter);
	sk_add_child(parent, sk);
	parent = sk;

	// Left Forearm & Hand (effector)
	sk = sk_alloc("ForeArmL");
	sk->is_effector = 1;
	mat4f_scale_new(sk->scale_mat, 0.2, THICKNESS, THICKNESS);
	mat4f_translate_new(sk->trans_mat, -0.21, 0, 0);
	//mat4f_mult_mat4f_new(sk->transform_matrix, trans_mat, scale_mat);
	mat4f_copy(sk->test_mat, decenter);
	sk_add_child(parent, sk);
	effectors[num_effectors].tracking_name = TRACKING_HANDL;
	effectors[num_effectors].effector = sk;
	num_effectors++;
	parent = sk;

	mat4f_translate_new(decenter, 0.5, 0, 0);

	// Right shoulder
	sk = sk_alloc("ShoulderR");
	sk->is_static = 1;
	mat4f_scale_new(sk->scale_mat, 0.2, THICKNESS, THICKNESS);
	mat4f_translate_new(sk->trans_mat, 0.19, 0, 0);
	// mat4f_mult_mat4f_new(sk->transform_matrix, trans_mat, scale_mat);
	mat4f_copy(sk->test_mat, decenter);
	sk_add_child(attach, sk);
	parent = sk;

	// Right Arm
	sk = sk_alloc("ArmR");
	// sk->angles[2] = 45;
	mat4f_scale_new(sk->scale_mat, 0.2, THICKNESS, THICKNESS);
	mat4f_translate_new(sk->trans_mat, 0.21, 0, 0);
	// mat4f_mult_mat4f_new(sk->transform_matrix, trans_mat, scale_mat);
	mat4f_copy(sk->test_mat, decenter);
	sk_add_child(parent, sk);
	parent = sk;

	// Right Forearm & Hand (effector)
	sk = sk_alloc("ForeArmR");
	sk->is_effector = 1;
	// sk->angles[2] = 45;
	mat4f_scale_new(sk->scale_mat, 0.2, THICKNESS, THICKNESS);
	mat4f_translate_new(sk->trans_mat, 0.21, 0, 0);
	// mat4f_mult_mat4f_new(sk->transform_matrix, trans_mat, scale_mat);
	mat4f_copy(sk->test_mat, decenter);
	sk_add_child(parent, sk);
	effectors[num_effectors].tracking_name = TRACKING_HANDR;
	effectors[num_effectors].effector = sk;
	num_effectors++;
	parent = sk;

	mat4f_translate_new(decenter, 0, -0.5, 0);

	// Torso
	sk = sk_alloc("Torso");
	sk->is_static = 1;
	mat4f_scale_new(sk->scale_mat, THICKNESS, 0.4, THICKNESS);
	mat4f_translate_new(sk->trans_mat, 0, -0.41, 0);
	mat4f_copy(sk->test_mat, decenter);
	sk_add_child(attach, sk);
	attach = sk;
	parent = sk;

	// Left Leg
	sk = sk_alloc("LegL");
	mat4f_scale_new(sk->scale_mat, THICKNESS, 0.2, THICKNESS);
	mat4f_translate_new(sk->trans_mat, 0, -0.21, 0);
	mat4f_copy(sk->test_mat, decenter);
	sk_add_child(attach, sk);
	parent = sk;

	// Left Foreleg
	sk = sk_alloc("ForeLegL");
	sk->is_effector = 1;
	mat4f_scale_new(sk->scale_mat, THICKNESS, 0.2, THICKNESS);
	mat4f_translate_new(sk->trans_mat, 0, -0.21, 0);
	mat4f_copy(sk->test_mat, decenter);
	sk_add_child(parent, sk);
	effectors[num_effectors].tracking_name = TRACKING_FOOTL;
	effectors[num_effectors].effector = sk;
	num_effectors++;
	parent = sk;

	// Right Leg
	sk = sk_alloc("LegR");
	mat4f_scale_new(sk->scale_mat, THICKNESS, 0.2, THICKNESS);
	mat4f_translate_new(sk->trans_mat, 0, -0.21, 0);
	mat4f_copy(sk->test_mat, decenter);
	sk_add_child(attach, sk);
	parent = sk;

	// Right Foreleg
	sk = sk_alloc("ForeLegR");
	sk->is_effector = 1;
	mat4f_scale_new(sk->scale_mat, THICKNESS, 0.2, THICKNESS);
	mat4f_translate_new(sk->trans_mat, 0, -0.21, 0);
	mat4f_copy(sk->test_mat, decenter);
	sk_add_child(parent, sk);
	effectors[num_effectors].tracking_name = TRACKING_FOOTR;
	effectors[num_effectors].effector = sk;
	num_effectors++;
	parent = sk;

	ik_init(ik);

	ik_compute_positions(ik);

	// Set targets to base positions.
	for (int i = 0; i < num_effectors; i++) {
		struct kuhl_skeleton *eff = effectors[i].effector;
		ik_set_effector_target(ik, eff, eff->position);
	}

	printf("Original positions:\n");
	sk = ik->sk;
	while (sk) {
		printf("%s: (%.2f, %.2f, %.2f)\n", sk->name,
			sk->position[0], sk->position[1], sk->position[2]);
		sk = sk_next(sk);
	}

	// Position when the shoulder and elbow joints are at 45 degrees.
	// float target[] = { 0.34, 0.25, 0.00 };
	// ik_set_effector_target(ik, effectors[1].effector, target);
}

void draw_model(float view_mat[16])
{
	float modelview[16];
	struct kuhl_skeleton *sk = ik->sk;

	while (sk) {
		if (sk->parent) {
			mat4f_mult_mat4f_new(modelview, view_mat, sk->parent->transform_matrix);
		} else {
			mat4f_copy(modelview, view_mat);
		}

		mat4f_mult_mat4f_new(modelview, modelview, sk->joint_matrix);

		// Draw the end effector positions for debugging purposes.
		if (sk->is_effector) {
			float scale[16];
			float trans[16];
			mat4f_scale_new(scale, THICKNESS+0.01, THICKNESS+0.01, THICKNESS+0.01);
			mat4f_translate_new(trans, sk->position[0], sk->position[1], sk->position[2]);
			mat4f_mult_mat4f_new(trans, view_mat, trans);
			mat4f_mult_mat4f_new(trans, trans, scale);
			glUniformMatrix4fv(kuhl_get_uniform("ModelView"),
			                   1, // number of 4x4 float matrices
			                   0, // transpose
			                   trans); // value
			kuhl_errorcheck();
			kuhl_geometry_draw(modelgeom); /* Draw the model */
			kuhl_errorcheck();
		}

		mat4f_mult_mat4f_new(modelview, modelview, sk->scale_mat);
		mat4f_mult_mat4f_new(modelview, modelview, sk->test_mat);

		// mat4f_mult_mat4f_new(modelview, modelview, sk->composite_matrix);
		// mat4f_mult_mat4f_new(modelview, modelview, sk->scale_mat);
		glUniformMatrix4fv(kuhl_get_uniform("ModelView"),
		                   1, // number of 4x4 float matrices
		                   0, // transpose
		                   modelview); // value
		kuhl_errorcheck();
		kuhl_geometry_draw(modelgeom); /* Draw the model */
		kuhl_errorcheck();

		sk = sk_next(sk);
	}
}


/** Draws the 3D scene. */
void display()
{
	/* Ensure the slaves use the same render style as the master
	 * process. */
	dgr_setget("style", &renderStyle, sizeof(int));


	/* Render the scene once for each viewport. Frequently one
	 * viewport will fill the entire screen. However, this loop will
	 * run twice for HMDs (once for the left eye and once for the
	 * right). */
	viewmat_begin_frame();
	for(int viewportID=0; viewportID<viewmat_num_viewports(); viewportID++)
	{
		viewmat_begin_eye(viewportID);

		/* Where is the viewport that we are drawing onto and what is its size? */
		int viewport[4]; // x,y of lower left corner, width, height
		viewmat_get_viewport(viewport, viewportID);
		/* Tell OpenGL the area of the window that we will be drawing in. */
		glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);

		/* Clear the current viewport. Without glScissor(), glClear()
		 * clears the entire screen. We could call glClear() before
		 * this viewport loop---but in order for all variations of
		 * this code to work (Oculus support, etc), we can only draw
		 * after viewmat_begin_eye(). */
		glScissor(viewport[0], viewport[1], viewport[2], viewport[3]);
		glEnable(GL_SCISSOR_TEST);
		glClearColor(.2,.2,.2,0); // set clear color to grey
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		glDisable(GL_SCISSOR_TEST);
		glEnable(GL_DEPTH_TEST); // turn on depth testing
		kuhl_errorcheck();

		/* Turn on blending (note, if you are using transparent textures,
		   the transparency may not look correct unless you draw further
		   items before closer items.). */
		glEnable(GL_BLEND);
		glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
		glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ZERO);

		/* Get the view or camera matrix; update the frustum values if needed. */
		float viewMat[16], perspective[16];
		viewmat_get(viewMat, perspective, viewportID);

		glUseProgram(program);

		glUniform1i(kuhl_get_uniform("renderStyle"), renderStyle);

		kuhl_errorcheck();
		/* Send the perspective projection matrix to the vertex program. */
		glUniformMatrix4fv(kuhl_get_uniform("Projection"),
		                   1, // number of 4x4 float matrices
		                   0, // transpose
		                   perspective); // value

//		float modelMat[16];
//		get_model_matrix(modelMat);
//		mat4f_stack_mult(stack, modelMat);

		// Get the effector location from vrpn
		if (USE_VRPN)
		{
			float orient[16];
			vrpn_get(TRACKING_HANDR, VRPN_HOST, target, orient);
			printf("Tracking position for %s: (%.2f, %.2f, %.2f)\n",
				   TRACKING_HANDR, target[0], target[1], target[2]);
		}

		// effector_target(target);

		// float arm1Mat[16],arm2Mat[16];
		// get_arm_matrices(arm1Mat, arm2Mat, angles);

		// float modelview[16];
		// mat4f_mult_mat4f_new(modelview, viewMat, arm1Mat);
		// glUniformMatrix4fv(kuhl_get_uniform("ModelView"),
		//                    1, // number of 4x4 float matrices
		//                    0, // transpose
		//                    modelview); // value
		// kuhl_errorcheck();
		// kuhl_geometry_draw(modelgeom);
		// kuhl_errorcheck();

		// mat4f_mult_mat4f_new(modelview, viewMat, arm2Mat);
		// glUniformMatrix4fv(kuhl_get_uniform("ModelView"),
		//                    1, // number of 4x4 float matrices
		//                    0, // transpose
		//                    modelview); // value
		// kuhl_errorcheck();
		// kuhl_geometry_draw(modelgeom);
		// kuhl_errorcheck();

		// float ealoc[4];
		// end_effector_loc(ealoc, arm2Mat);

		int iterations = ik_jacobian_transpose(ik, 0.005, 100, 2);
		if (iterations) {
			printf("IK iterations: %d\n", iterations);
		}

		draw_model(viewMat);

		glUseProgram(0); // stop using a GLSL program.

		static int counter = 0;
		counter++;
		if(counter % 60 == 0)
			msg(MSG_INFO, "FPS: %0.2f\n", bufferswap_fps());

		viewmat_end_eye(viewportID);
	} // finish viewport loop
	viewmat_end_frame();

	/* Update the model for the next frame based on the time. We
	 * convert the time to seconds and then use mod to cause the
	 * animation to repeat. */
	double time = glfwGetTime();
	dgr_setget("time", &time, sizeof(double));
	kuhl_update_model(modelgeom, 0, fmod(time,10));

	/* Check for errors. If there are errors, consider adding more
	 * calls to kuhl_errorcheck() in your code. */
	kuhl_errorcheck();

	//kuhl_video_record("videoout", 30);
}


int main(int argc, char** argv)
{
	/* Initialize GLFW and GLEW */
	kuhl_ogl_init(&argc, argv, 512, 512, 32, 4);

	char *modelFilename    = "../models/cube/cube.obj";
	char *modelTexturePath = NULL;

	/* Specify function to call when keys are pressed. */
	glfwSetKeyCallback(kuhl_get_window(), keyboard);
	// glfwSetFramebufferSizeCallback(window, reshape);

	/* Compile and link a GLSL program composed of a vertex shader and
	 * a fragment shader. */
	program = kuhl_create_program(GLSL_VERT_FILE, GLSL_FRAG_FILE);

	dgr_init();     /* Initialize DGR based on environment variables. */

	float initCamPos[3]  = {0,1.55,2}; // 1.55m is a good approx eyeheight
	float initCamLook[3] = {0,0,0}; // a point the camera is facing at
	float initCamUp[3]   = {0,1,0}; // a vector indicating which direction is up
	viewmat_init(initCamPos, initCamLook, initCamUp);

	// Clear the screen while things might be loading
	glClearColor(.2,.2,.2,1);
	glClear(GL_COLOR_BUFFER_BIT);

	// Load the model from the file
	modelgeom = kuhl_load_model(modelFilename, modelTexturePath, program, bbox);
	if(modelgeom == NULL)
	{
		msg(MSG_FATAL, "Unable to load the requested model: %s", modelFilename);
		exit(EXIT_FAILURE);
	}

	init_model();

	while(!glfwWindowShouldClose(kuhl_get_window()))
	{
		display();
		kuhl_errorcheck();

		/* process events (keyboard, mouse, etc) */
		glfwPollEvents();
	}
	exit(EXIT_SUCCESS);
}
