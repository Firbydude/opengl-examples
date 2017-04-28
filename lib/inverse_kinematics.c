#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "list.h"
#include "libkuhl.h"

#include "inverse_kinematics.h"

// Index into the jacobian for effector index i, joint j, & axis a.
#define J(ik, i, j, a) ((j)*(ik)->num_effectors*3*3 \
					  + (a)*(ik)->num_effectors*3 \
					  + (i)*3)

struct kuhl_skeleton *sk_alloc(const char *name) {
	struct kuhl_skeleton *sk = malloc(sizeof(struct kuhl_skeleton));

	// Poor mans init
	bzero(sk, sizeof(struct kuhl_skeleton));

	if (name != NULL) {
		sk->name = strdup(name);
	}

	return sk;
}

void sk_add_child(struct kuhl_skeleton *sk, struct kuhl_skeleton *child)
{
	struct kuhl_skeleton **temp;

	temp = malloc(sizeof(struct kuhl_skeleton*) * (sk->num_children + 1));

	if (sk->num_children > 0) {
		for (int i = 0; i < sk->num_children; i++) {
			temp[i] = sk->children[i];
		}

		free(sk->children);
	}

	sk->children = temp;
	sk->children[sk->num_children] = child;
	sk->num_children++;

	child->parent = sk;
}

int sk_rem_child(struct kuhl_skeleton *sk, struct kuhl_skeleton *child)
{
	int shift = 0;

	for (int i = 0; i < sk->num_children; i++) {
		if (shift) {
			sk->children[i-1] = sk->children[i];
		} else if (sk->children[i] == child) {
			shift = 1;
		}
	}

	if (shift) {
		sk->num_children--;
		child->parent = NULL;
	}

	return shift;
}

struct kuhl_skeleton *
sk_next_child(struct kuhl_skeleton *sk, struct kuhl_skeleton *child)
{
	if (sk == NULL) {
		return NULL;
	}

	for (int i = 0; i < sk->num_children; i++) {
		if (sk->children[i] == child) {
			if (i + 1 < sk->num_children) {
				return sk->children[i + 1];
			} else {
				return sk_next_child(sk->parent, sk);
			}
		}
	}

	return NULL;
}

struct kuhl_skeleton *sk_next(struct kuhl_skeleton *sk)
{
	if (sk->num_children > 0) {
		return sk->children[0];
	}

	if (sk->parent) {
		return sk_next_child(sk->parent, sk);
	}

	return NULL;
}

struct kuhl_ik *ik_alloc()
{
	struct kuhl_ik *ik;
	ik = malloc(sizeof(struct kuhl_ik));
	bzero(ik, sizeof(struct kuhl_ik));

	return ik;
}

void ik_init(struct kuhl_ik *ik)
{
	ik->num_effectors = 0;
	ik->num_joints = 0;

	if (ik->effectors) { list_free(ik->effectors); }
	if (ik->eff_targets) { free(ik->eff_targets); }
	if (ik->jacobian) { free(ik->jacobian); }
	if (ik->delta_angles) { free(ik->delta_angles); }

	ik->effectors = list_new(16, sizeof(void*), NULL);

	struct kuhl_skeleton *sk = ik->sk;
	while (sk) {
		if (sk->is_effector) {
			sk->eindex = ik->num_effectors++;
			list_append(ik->effectors, &sk);
		}
		
		// Effectors also count as joints.
		sk->jindex = ik->num_joints++;

		sk = sk_next(sk);
	}

	ik->eff_targets = malloc(sizeof(float) * ik->num_effectors * 3);
	ik->jacobian = malloc(sizeof(float)
						* ik->num_effectors * 3
						* ik->num_joints * 3);
	ik->delta_angles = malloc(sizeof(float) * ik->num_joints * 3);
}

// static void _ik_upcompute_position(struct kuhl_skeleton *sk, float *pos) {
// 	float transform[16];

// 	if (sk->parent) {
// 		_ik_upcompute_position
// 	}
// }

static void _ik_transpose(float *result, float *matrix, int cols, int rows)
{
	for (int c = 0; c < cols; c++) {
		for (int r = 0; r < rows; r++) {
			// result[c][r] = matrix[r][c]
			result[r*cols + c] = matrix[c*rows + r];
		}
	}
}

// static void ik__compute_eff_position(struct kuhl_skeleton *sk, 
// 	struct kuhl_skeleton *eff)
// {

// }

static void _ik_compute_position(struct kuhl_skeleton *sk)
{
	mat4f_rotateEuler_new(sk->joint_matrix, sk->angles[0], sk->angles[1],
						  sk->angles[2], "XYZ");

	bzero(sk->position, sizeof(float) * 3);
	sk->position[3] = 1;

	if (sk->parent) {
		struct kuhl_skeleton *p = sk->parent;
		mat4f_mult_mat4f_new(sk->transform_matrix, p->transform_matrix, sk->joint_matrix);
		mat4f_mult_mat4f_new(sk->transform_matrix, sk->transform_matrix, sk->trans_mat);
	} else {
		mat4f_mult_mat4f_new(sk->transform_matrix, sk->joint_matrix, sk->trans_mat);
	}

	mat4f_mult_vec4f(sk->position, sk->transform_matrix);

	for (int i = 0; i < sk->num_children; i++) {
		_ik_compute_position(sk->children[i]);
	}
}

void ik_compute_positions(struct kuhl_ik *ik)
{
	_ik_compute_position(ik->sk);
}

void ik_compute_jacobian(struct kuhl_ik *ik, float delta)
{
	int ji = 0; // joint index
	int ei = 0; // effector index
	struct kuhl_skeleton *eff;
	struct kuhl_skeleton *joint;
	// float d[3] = { delta, delta, delta };
	float delta_pos[3];

	bzero(ik->jacobian,
		sizeof(float) * ik->num_effectors * 3 * ik->num_joints * 3);

	for (ei = 0; ei < ik->num_effectors; ei++) {
		if (!list_get(ik->effectors, ei, &eff)) {
			printf("PANIC!!!\n");
		}

		joint = eff;
		do {
			// Don't include static joints, it'll through off the inverse.
			if (joint->is_static) {
				joint = joint->parent;
				continue;
			}

			// TODO Joint constraint stuff

			// TODO experiment with the cross product method.
			// vec3f_add(joint->angles, d);

			// _ik_upcompute_position(eff, delta_pos);
			// vec3f_sub_new(jacobian)

			// vec3f_sub_new(joint->angles, joint->angles, d);

			ji = joint->jindex;
			for (int a = 0; a < 3; a++) {
				vec3f_copy(delta_pos, eff->position);

				joint->angles[a] += delta;

				// TODO This is inefficient, come back later
				_ik_compute_position(joint);


				vec3f_sub_new(delta_pos, eff->position, delta_pos);
				int index = J(ik, ei, ji, a);
				if (index >= ik->num_effectors * 3 * ik->num_joints * 3)
					printf("Ji > size\n");
				vec3f_copy(&ik->jacobian[index], delta_pos);
				joint->angles[a] -= delta;

				// Reset transform matrices
				_ik_compute_position(joint);
			}

			joint = joint->parent;
		} while (joint && joint->parent);
	}

	#ifdef DEBUG
	float transpose[ik->num_effectors * 3 * ik->num_joints * 3];
	_ik_transpose(transpose, ik->jacobian, ik->num_joints * 3, ik->num_effectors * 3);
	printf("Jacobian:\n");
	for(int i = 0; i < ik->num_effectors * 3; i++)
	{
		for (int a = 0; a < ik->num_joints * 3; ei++) {
			printf("%4.2f ", transpose[i * ik->num_joints * 3 + a]);
		}

		printf("\n");
	}
	#endif
}

void ik_get_effector_target(struct kuhl_ik *ik,
							struct kuhl_skeleton *eff,
							float *target)
{
	vec3f_copy(target, &ik->eff_targets[eff->eindex * 3]);
}

void ik_set_effector_target(struct kuhl_ik *ik,
							struct kuhl_skeleton *eff,
							float *target)
{
	vec3f_copy(&ik->eff_targets[eff->eindex * 3], target);
}

int ik_jacobian_transpose(struct kuhl_ik *ik,
						   float cutoff,
						   int max_iterations,
						   float delta)
{
	int iterations = 0;
	float delta_targets[ik->num_effectors * 3];
	float delta_angles[ik->num_joints * 3];
	float delta_expected[ik->num_effectors * 3]; // Expected change in position
	float transpose[ik->num_effectors * 3 * ik->num_joints * 3];
	struct kuhl_skeleton *sk;

	while (iterations < max_iterations) {
		float max_dist = 0;

		// Compute distance to targets
		for (int ei = 0; ei < ik->num_effectors; ei++) {
			struct kuhl_skeleton *eff;
			list_get(ik->effectors, ei, &eff);
			float *eff_pos = eff->position;
			vec3f_sub_new(&delta_targets[ei*3], &ik->eff_targets[ei*3], eff_pos);

			float dist = vec3f_norm(&delta_targets[ei*3]);
			if (dist > max_dist)
				max_dist = dist;

			#ifdef DEBUG
			printf("Effector %s dist: %.3f\n", eff->name, dist);
			#endif
		}

		// Cutoff using the maximum distance. Using the avg may also be ok.
		if (max_dist < cutoff) {
			return iterations;
		}

		ik_compute_jacobian(ik, delta);

		_ik_transpose(transpose, ik->jacobian,
			ik->num_joints * 3, ik->num_effectors * 3);

		// Multiply the change in the end effectors by the transposed
		// Jacobian. The result will approximate the change in angle
		// of each of the joints that we want.
		// **Since the jacobian is already in column major ordering
		// **we can just dot the columns with the vector.
		for (int a = 0; a < ik->num_joints * 3; a++) {
			delta_angles[a] = vecNf_dot(
				&ik->jacobian[a * ik->num_effectors * 3],
				delta_targets,
				ik->num_effectors * 3);
		}

		// Calculate how these changes in angle would influence the end
		// effectors based on the original Jacobian.
		for (int a = 0; a < ik->num_effectors * 3; a++) {
			// Dot product rows of Jacobian with angle deltas.
			// We use the columns of the transpose here for easier access.
			delta_expected[a] = vecNf_dot(
				&transpose[ik->num_joints * 3 * a], 
				delta_angles, 
				ik->num_joints * 3);
		}

		#ifdef DEBUG
		printf("Expected change in effector:\n");
		char str[256];
		vecNf_print_to_string(str, 256, delta_expected, ik->num_effectors * 3);
		printf("%s\n", str);
		#endif

		// Calculate a reasonable alpha according to:
		// http://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
		float alpha = vecNf_dot(delta_expected, delta_targets, ik->num_effectors * 3) /
					  vecNf_dot(delta_expected, delta_expected, ik->num_effectors * 3);
		#ifdef DEBUG
		printf("Alpha: %f\n", alpha);
		#endif

		// Apply our angle changes (multiplied by alpha) to the skeleton's joints.
		for(int i = 0; i < ik->num_joints * 3; i++) {
			delta_angles[i] *= alpha;
		}

		sk = ik->sk;
		while (sk) {
			if (!sk->is_static) {
				for (int i = 0; i < 3; i++) {
					sk->angles[i] += delta_angles[sk->jindex * 3 + i];
					// Keep angles within 0 to 360
					sk->angles[i] = fmod(sk->angles[i], 360);
				}
			}

			sk = sk_next(sk);
		}

		#ifdef DEBUG
		printf("Change in angles: ");
		for (int i = 0; i < ik->num_joints * 3; i++) {
			printf("%.2f ", delta_angles[i]);
		}
		printf("\n");
		#endif

		ik_compute_positions(ik);

		// float newLoc[4];
		// end_effector_loc(newLoc, angles);
		// float actualChange[3];
		// vec3f_sub_new(actualChange, newLoc, currentLoc);
		// printf("Actual change in end effector\n");
		// vec3f_print(actualChange);

		iterations++;
	}

	return iterations;
}
