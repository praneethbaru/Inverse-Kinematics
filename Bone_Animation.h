#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>	

class Bone_Animation
{
public:
	Bone_Animation();
	~Bone_Animation();

	void init();
	void update(float delta_time);
	void reset();

	glm::mat4 rotate_1(glm::mat4 bone, glm::vec3 rotationVector);
	glm::mat4 rotate_3(glm::mat4 bone, glm::vec3 rotationVector);
	glm::mat4 rotate_2(glm::mat4 bone, glm::vec3 rotationVector);

	glm::vec3 rotateEndEffector_1(glm::vec3 endEffector, glm::mat4 bone, glm::vec3 rotationVector);
	glm::vec3 rotateEndEffector_2(glm::vec3 endEffector, glm::mat4 bone, glm::vec3 rotationVector);
	glm::vec3 rotateEndEffector_3(glm::vec3 endEffector, glm::mat4 bone, glm::vec3 rotationVector);

	glm::vec3 jacobian_bone_1(glm::vec3 target_position, glm::vec3 end_effector, glm::vec3 angle_bone_1);
	glm::vec3 jacobian_bone_2(glm::vec3 end_effector1, glm::vec3 end_effector2, glm::vec3 angle_bone_2);
	glm::vec3 jacobian_bone_3(glm::vec3 end_effector2, glm::vec3 end_effector3, glm::vec3 angle_bone_2);

	glm::mat4 JacobianRotation1(glm::vec3 goal, glm::vec3 endEffector, glm::vec3 angle_bone_1);
	glm::mat4 JacobianRotation2(glm::vec3 goal, glm::vec3 endEffector, glm::vec3 rotationaAngle);
	glm::mat4 JacobianRotation3(glm::vec3 goal, glm::vec3 endEffector, glm::vec3 rotationaAngle);


	glm::vec3 getDeltaAngle(float alpha, glm::mat3 jacobTranspose, glm::vec3 target);
	glm::mat3 CalculateAxisRotation(glm::mat4 parent_matrix, glm::vec3 rotationAngle);
	float getAlpha(glm::mat3 jacobMat, glm::mat3 jacobTranspose, glm::vec3 target);

	glm::mat4 get_target_mat()		{ return target_mat; }
	glm::mat4 get_root_bone_mat()	{ return root_bone_mat; }
	glm::mat4 get_bone_1_mat()		{ return bone_1_mat; }
	glm::mat4 get_bone_2_mat()		{ return bone_2_mat; }
	glm::mat4 get_bone_3_mat()		{ return bone_3_mat; }

public:

	// Here the head of each vector is the root bone
	float beta = 0.65f;

	std::vector<glm::vec3> scale_vector;
	std::vector<glm::vec3> rotation_degree_vector;
	std::vector<glm::vec4> colors;

	glm::vec3 target_position;
	glm::vec3 end_effector1;
	glm::vec3 end_effector2;
	glm::vec3 end_effector3;

	glm::vec3 initial_bone_1_rotation;
	glm::vec3 initial_bone_2_rotation;
	glm::vec3 initial_bone_3_rotation;

	glm::vec3 initial_end_effector1;
	glm::vec3 initial_end_effector2;
	glm::vec3 initial_end_effector3;



	glm::vec3 root_position;
	glm::vec3 total_bone_1;
	glm::vec3 total_bone_2;
	glm::vec3 total_bone_3;

	glm::vec3 joint_root = { 2.0f, 1.0f, 2.0f };
	glm::vec3 joint_1;
	glm::vec3 joint_2;
	glm::vec3 joint_3;

	glm::vec3 bone_1_rotation;
	glm::vec3 bone_2_rotation;
	glm::vec3 bone_3_rotation;

	glm::mat4 x;
	glm::mat4 y;
	glm::mat4 z;

	glm::mat3 x3;
	glm::mat3 y3;
	glm::mat3 z3;

	glm::mat4 target_mat;
	glm::mat4 root_bone_mat;
	glm::mat4 bone_1_mat;
	glm::mat4 bone_2_mat;
	glm::mat4 bone_3_mat;

};
