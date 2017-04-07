#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "list.h"
#include "libkuhl.h"

#include "inverse_kinematics.h"

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
			ik->num_effectors++;
			list_append(ik->effectors, sk);
		} else {
			ik->num_joints++;
		}

		sk = sk_next(sk);
	}

	ik->eff_targets = malloc(sizeof(float) * ik->num_effectors * 3);
	ik->jacobian = malloc(sizeof(float)
						* ik->num_effectors * 3
						* ik->num_joints * 3);
	ik->delta_angles = malloc(sizeof(float) * ik->num_joints * 3);
}

void ik_compute_jacobian(struct kuhl_ik *ik)
{

}

static void _ik_compute_position(struct kuhl_skeleton *sk)
{
	static float position_matrix[16];

	mat4f_rotateEuler_new(sk->joint_matrix, sk->angles[0], sk->angles[1],
						  sk->angles[2], "XYZ");

	if (sk->parent) {
		struct kuhl_skeleton *p = sk->parent;
		mat4f_mult_mat4f_new(sk->transform_matrix, p->transform_matrix, sk->trans_mat);
		mat4f_mult_mat4f_new(sk->transform_matrix, sk->transform_matrix, sk->joint_matrix);
		mat4f_mult_mat4f_new(sk->composite_matrix, sk->transform_matrix, sk->scale_mat);

		// mat4f_mult_mat4f_new(sk->composite_matrix,
		// 					 sk->joint_matrix,
		// 					 sk->parent->joint_matrix);
	} else {
		memcpy(sk->composite_matrix, sk->joint_matrix, sizeof(float) * 16);
		mat4f_mult_mat4f_new(sk->transform_matrix, sk->trans_mat, sk->joint_matrix);
		mat4f_mult_mat4f_new(sk->composite_matrix, sk->transform_matrix, sk->scale_mat);
	}

	bzero(sk->position, sizeof(float) * 3);
	sk->position[3] = 1;
	mat4f_mult_mat4f_new(position_matrix, sk->composite_matrix, sk->transform_matrix);
	mat4f_mult_vec4f(sk->position, position_matrix);

	for (int i = 0; i < sk->num_children; i++) {
		_ik_compute_position(sk->children[i]);
	}
}

void ik_compute_positions(struct kuhl_ik *ik)
{
	_ik_compute_position(ik->sk);
}
