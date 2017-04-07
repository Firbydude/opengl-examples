#ifndef _INVERSE_KINEMATICS_H_
#define _INVERSE_KINEMATICS_H_

#include "list.h"

/**
 * Holds some information about a kuhl_skeleton for doing inverse kinematic
 * operations. Call kuhl_ik_init when finished creating the skeleton. Updates
 * to the skeleton do NOT automatically reflect in the kuhl_ik struct. Call
 * kuhl_ik_init after making any structural changes to a skeleton.
 */
struct kuhl_ik {
	struct kuhl_skeleton *sk;

	int num_effectors; // K
	int num_joints; // N

	list *effectors;

	float *eff_targets;

	// A (K*3)X(N*3) matrix J
	float *jacobian;	  // J
	float *delta_angles;
};

struct kuhl_skeleton {
	char *name;

	int index;
	int is_effector;
	int is_static; // Static joint angles cannot be modified.

	float position[4];
	float orientation[16];
	float angles[3];
	float constraints[6];

	float scale_mat[16];
	float trans_mat[16];

	float transform_matrix[16];
	float joint_matrix[16];
	float composite_matrix[16];

	struct kuhl_skeleton *parent;

	int num_children;
	struct kuhl_skeleton **children;

	// This will probably end up being kuhl_geometry or aiBone.
	void *data;
};

struct kuhl_skeleton *sk_alloc(const char *name);

void sk_add_child(struct kuhl_skeleton *sk, struct kuhl_skeleton *child);

int sk_rem_child(struct kuhl_skeleton *sk, struct kuhl_skeleton *child);

struct kuhl_skeleton *sk_next(struct kuhl_skeleton *sk);

void ik_compute_jacobian(struct kuhl_ik *ik);

void ik_compute_positions(struct kuhl_ik *ik);

void ik_init(struct kuhl_ik *ik);

struct kuhl_ik *ik_alloc();

#endif
