/*********************************************************************
*  Inverse Kinematics Program.
*
*  This program implements Inverse Kinematics on 3 bones having 3 joints
*  and a total of 9 DOFs. The end effector iteratively moves towards the
*  target using the Jacobian Transpose method.
*
*  Each iteration determines a fraction of angle with which each bone has 
*  to rotate to reach the target position.
*	
*  Written by Praneeth Kumar Baru
*  Date strated: 10/29/2018
*********************************************************************/
#include "Bone_Animation.h"
#include "windows.h"

int i, j, k;

Bone_Animation::Bone_Animation()
{
}


Bone_Animation::~Bone_Animation()
{
}

void Bone_Animation::init()
{
	root_position = { 2.0f,0.0f,2.0f };
	target_position = {3.0f,8.0f, 3.0f };

	scale_vector =
	{
		{1.0f,1.0f,1.0f},
		{0.5f,4.0f,0.5f},
		{0.5f,3.0f,0.5f},
		{0.5f,2.0f,0.5f}
	};

	rotation_degree_vector =
	{
		{0.0f,0.0f,0.0f},
		{0.0f,0.0f,0.0f},
		{0.0f,0.0f,0.0f},
		{0.0f,0.0f,0.0f}
	};

	colors =
	{
		{0.0f,0.7f,0.0f,1.0f},
		{0.7f,0.0f,0.0f,1.0f},
		{0.7f,0.7f,0.0f,1.0f},
		{0.7f,0.0f,0.7f,1.0f},
		{0.0f,0.7f,0.7f,1.0f}
	};

	reset();

}

void Bone_Animation::update(float delta_time)
{
}


/*****************************************************************
* This function rotates any given matrix passed
* as parameter in given x,y,z orientations relative to BONE 2.
*****************************************************************/
glm::mat4 Bone_Animation::rotate_3(glm::mat4 bone, glm::vec3 rotationVector)
{
	glm::mat4 result_mat;
	float angle_x, angle_y, angle_z;

	angle_x = glm::radians(rotationVector.x);
	angle_y = glm::radians(rotationVector.y);
	angle_z = glm::radians(rotationVector.z);

	// Translating to the joint - tip of Bone 2
	bone[3][0] = bone[3][0] - end_effector2[0];
	bone[3][1] = bone[3][1] - end_effector2[1];
	bone[3][2] = bone[3][2] - end_effector2[2];

	// Rotating Y
	y = {
			{cos(angle_y), 0, -sin(angle_y), 0},
			{ 0, 1, 0, 0 },
			{ sin(angle_y), 0, cos(angle_y),0},
			{0, 0, 0, 1}
	};

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_mat[i][j] = 0;
			for (k = 0; k < 4; k++)
				result_mat[i][j] += bone[i][k] * y[k][j];
		}
	}

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
		{
			bone[i][j] = result_mat[i][j];
		}
	}

	//Rotating Z
	z = {
		{cos(angle_z), sin(angle_z), 0, 0},
		{ -sin(angle_z), cos(angle_z), 0, 0 },
		{ 0, 0, 1, 0},
		{0, 0, 0, 1}
	};

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_mat[i][j] = 0;
			for (k = 0; k < 4; k++)
				result_mat[i][j] += bone[i][k] * z[k][j];
		}
	}

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
		{
			bone[i][j] = result_mat[i][j];
		}
	}

	// Rotating X
	x = {
			{1, 0, 0, 0},
			{ 0, cos(angle_x), sin(angle_x), 0 },
			{ 0, -sin(angle_x), cos(angle_x),0},
			{0, 0, 0, 1}
	};

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_mat[i][j] = 0;
			for (k = 0; k < 4; k++)
				result_mat[i][j] += bone[i][k] * x[k][j];
		}
	}

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
		{
			bone[i][j] = result_mat[i][j];
		}
	}

	//Translating to its new position
	bone[3][0] = bone[3][0] + end_effector2[0];
	bone[3][1] = bone[3][1] + end_effector2[1];
	bone[3][2] = bone[3][2] + end_effector2[2];

	return bone;
}

/*****************************************************************
* This function rotates any given matrix passed
* as parameter in given x,y,z orientations relative to BONE 1.
*****************************************************************/

glm::mat4 Bone_Animation::rotate_2(glm::mat4 bone, glm::vec3 rotationVector)
{
	glm::mat4 result_mat;
	float angle_x, angle_y, angle_z;

	angle_x = glm::radians(rotationVector.x);
	angle_y = glm::radians(rotationVector.y);
	angle_z = glm::radians(rotationVector.z);

	// Translating to the joint - tip of Bone 1
	bone[3][0] = bone[3][0] -end_effector1[0];
	bone[3][1] = bone[3][1] - end_effector1[1];
	bone[3][2] = bone[3][2] - end_effector1[2];

	// Rotating Y
	y = {
			{cos(angle_y), 0, -sin(angle_y), 0},
			{ 0, 1, 0, 0 },
			{ sin(angle_y), 0, cos(angle_y),0},
			{0, 0, 0, 1}
	};

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_mat[i][j] = 0;
			for (k = 0; k < 4; k++)
				result_mat[i][j] += bone[i][k] * y[k][j];
		}
	}

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
		{
			bone[i][j] = result_mat[i][j];
		}
	}

	// Rotating Z
	z = {
		{cos(angle_z), sin(angle_z), 0, 0},
		{ -sin(angle_z), cos(angle_z), 0, 0 },
		{ 0, 0, 1, 0},
		{0, 0, 0, 1}
	};

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_mat[i][j] = 0;
			for (k = 0; k < 4; k++)
				result_mat[i][j] += bone[i][k] * z[k][j];
		}
	}

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
		{
			bone[i][j] = result_mat[i][j];
		}
	}

	// Rotating X
	x = {
			{1, 0, 0, 0},
			{ 0, cos(angle_x), sin(angle_x), 0 },
			{ 0, -sin(angle_x), cos(angle_x),0},
			{0, 0, 0, 1}
	};

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_mat[i][j] = 0;
			for (k = 0; k < 4; k++)
				result_mat[i][j] += bone[i][k] * x[k][j];
		}
	}

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
		{
			bone[i][j] = result_mat[i][j];
		}
	}

	// Translating to new position
	bone[3][0] = bone[3][0] + end_effector1[0];
	bone[3][1] = bone[3][1] + end_effector1[1];
	bone[3][2] = bone[3][2] + end_effector1[2];

	return bone;
}

/*****************************************************************
* This function rotates any given matrix passed
* as parameter in given x,y,z orientations relative to ROOT.
*****************************************************************/

glm::mat4 Bone_Animation::rotate_1(glm::mat4 bone, glm::vec3 rotationVector)
{
	glm::mat4 result_mat;
	float angle_x, angle_y, angle_z;

	angle_x = glm::radians(rotationVector.x);
	angle_y = glm::radians(rotationVector.y);
	angle_z = glm::radians(rotationVector.z);

	// Translating to the joint - tip of ROOT
	bone[3][0] = bone[3][0] - root_bone_mat[3][0];
	bone[3][1] = bone[3][1] - root_bone_mat[3][1] - (scale_vector[0][1] / 2);
	bone[3][2] = bone[3][2] - root_bone_mat[3][2];

	// Rotating Y
	y = {
			{cos(angle_y), 0, -sin(angle_y), 0},
			{ 0, 1, 0, 0 },
			{ sin(angle_y), 0, cos(angle_y),0},
			{0, 0, 0, 1}
	};

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_mat[i][j] = 0;
			for (k = 0; k < 4; k++)
				result_mat[i][j] += bone[i][k] * y[k][j];
		}
	}

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
		{
			bone[i][j] = result_mat[i][j];
		}
	}

	// Rotating Z
	z = {
		{cos(angle_z), sin(angle_z), 0, 0},
		{ -sin(angle_z), cos(angle_z), 0, 0 },
		{ 0, 0, 1, 0},
		{0, 0, 0, 1}
	};

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_mat[i][j] = 0;
			for (k = 0; k < 4; k++)
				result_mat[i][j] += bone[i][k] * z[k][j];
		}
	}

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
		{
			bone[i][j] = result_mat[i][j];
		}
	}

	// Rotating X
	x = {
			{1, 0, 0, 0},
			{ 0, cos(angle_x), sin(angle_x), 0 },
			{ 0, -sin(angle_x), cos(angle_x),0},
			{0, 0, 0, 1}
	};

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_mat[i][j] = 0;
			for (k = 0; k < 4; k++)
				result_mat[i][j] += bone[i][k] * x[k][j];
		}
	}

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
		{
			bone[i][j] = result_mat[i][j];
		}
	}

	// Translating to new position
	bone[3][0] = bone[3][0] + root_bone_mat[3][0];
	bone[3][1] = bone[3][1] + root_bone_mat[3][1] + (scale_vector[0][1] / 2);
	bone[3][2] = bone[3][2] + root_bone_mat[3][2];

	return bone;
}



/*****************************************************************
* Determines the position of the joint between Bone 1 and 2.
*****************************************************************/
glm::vec3 Bone_Animation::rotateEndEffector_1(glm::vec3 endEffector, glm::mat4 bone, glm::vec3 rotationVector)
{
	float angle_x, angle_y, angle_z;
	glm::vec3 result_mat;
	//end_effector1 = endEffector;

	angle_x = glm::radians(rotationVector.x);
	angle_y = glm::radians(rotationVector.y);
	angle_z = glm::radians(rotationVector.z);

	// Translating to the joint - tip of ROOT // 2,1,2
	endEffector[0] = endEffector[0] - bone[3][0];
	endEffector[1] = endEffector[1] - bone[3][1] - 0.5f;
	endEffector[2] = endEffector[2] - bone[3][2];


	// Rotating Y
	y3 = {
			{cos(angle_y), 0, -sin(angle_y)},
			{ 0, 1, 0 },
			{ sin(angle_y), 0, cos(angle_y)}
	};

	for (i = 0; i < 3; i++)
	{
		result_mat[i] = 0.0f;
		for (j = 0; j < 3; j++)
		{
			result_mat[i] += endEffector[j] * y3[j][i];
		}
	}

	for (i = 0; i < 3; i++)
	{
		endEffector[i] = result_mat[i];
	}

	// Rotating Z
	z3 = {
		{cos(angle_z), sin(angle_z), 0},
		{ -sin(angle_z), cos(angle_z), 0 },
		{ 0, 0, 1}
	};

	for (i = 0; i < 3; i++)
	{
		result_mat[i] = 0.0f;
		for (j = 0; j < 3; j++)
		{
			result_mat[i] += endEffector[j] * z3[j][i];
		}
	}

	for (i = 0; i < 3; i++)
	{
		endEffector[i] = result_mat[i];
	}

	//Rotating X
	x3 = {
			{1, 0, 0},
			{ 0, cos(angle_x), sin(angle_x)},
			{ 0, -sin(angle_x), cos(angle_x)}
	};

	for (i = 0; i < 3; i++)
	{
		result_mat[i] = 0.0f;
		for (j = 0; j < 3; j++)
		{
			result_mat[i] += endEffector[j] * x3[j][i];
		}
	}

	for (i = 0; i < 3; i++)
	{
		endEffector[i] = result_mat[i];
	}

	endEffector[0] = endEffector[0] + bone[3][0];
	endEffector[1] = endEffector[1] + bone[3][1] + 0.5f;
	endEffector[2] = endEffector[2] + bone[3][2];

	return endEffector;
}

/*****************************************************************
* Determines the position of the joint between BONE 2 and 3.
*****************************************************************/
glm::vec3 Bone_Animation::rotateEndEffector_2(glm::vec3 endEffector, glm::mat4 bone, glm::vec3 rotationVector)
{
	glm::vec3 result_mat;
	float angle_x, angle_y, angle_z;

	angle_x = glm::radians(rotationVector.x);
	angle_y = glm::radians(rotationVector.y);
	angle_z = glm::radians(rotationVector.z);

	// Translating to the joint - tip of ROOT // 2,8,2
	endEffector[0] = endEffector[0] - end_effector1[0];
	endEffector[1] = endEffector[1] - end_effector1[1];
	endEffector[2] = endEffector[2] - end_effector1[2];

	// Rotating Y
	y3 = {
			{cos(angle_y), 0, -sin(angle_y)},
			{ 0, 1, 0 },
			{ sin(angle_y), 0, cos(angle_y)}
	};

	for (i = 0; i < 3; i++)
	{
		result_mat[i] = 0.0f;
		for (j = 0; j < 3; j++)
		{
			result_mat[i] += endEffector[j] * y3[j][i];
		}
	}

	for (i = 0; i < 3; i++)
	{
		endEffector[i] = result_mat[i];
	}

	// Rotating Z
	z3 = {
		{cos(angle_z), sin(angle_z), 0},
		{ -sin(angle_z), cos(angle_z), 0 },
		{ 0, 0, 1}
	};

	for (i = 0; i < 3; i++)
	{
		result_mat[i] = 0.0f;
		for (j = 0; j < 3; j++)
		{
			result_mat[i] += endEffector[j] * z3[j][i];
		}
	}

	for (i = 0; i < 3; i++)
	{
		endEffector[i] = result_mat[i];
	}

	// Rotating X
	x3 = {
			{1, 0, 0},
			{ 0, cos(angle_x), sin(angle_x)},
			{ 0, -sin(angle_x), cos(angle_x)}
	};

	for (i = 0; i < 3; i++)
	{
		result_mat[i] = 0.0f;
		for (j = 0; j < 3; j++)
		{
			result_mat[i] += endEffector[j] * x3[j][i];
		}
	}

	for (i = 0; i < 3; i++)
	{
		endEffector[i] = result_mat[i];
	}

	endEffector[0] = endEffector[0] + end_effector1[0];
	endEffector[1] = endEffector[1] + end_effector1[1];
	endEffector[2] = endEffector[2] + end_effector1[2];

	return endEffector;
}

/*****************************************************************
* Determines the position of the end effector.
*****************************************************************/
glm::vec3 Bone_Animation::rotateEndEffector_3(glm::vec3 endEffector, glm::mat4 bone, glm::vec3 rotationVector)
{
	glm::vec3 result_mat;
	float angle_x, angle_y, angle_z;

	angle_x = glm::radians(rotationVector.x);
	angle_y = glm::radians(rotationVector.y);
	angle_z = glm::radians(rotationVector.z);

	// Translating to the joint - tip of ROOT // 2,8,2
	endEffector[0] = endEffector[0] - end_effector2[0];
	endEffector[1] = endEffector[1] - end_effector2[1];
	endEffector[2] = endEffector[2] - end_effector2[2];


	result_mat = { 0.0f ,0.0f,0.0f };

	// Rotating Y
	y3 = {
			{cos(angle_y), 0, -sin(angle_y)},
			{ 0, 1, 0 },
			{ sin(angle_y), 0, cos(angle_y)}
	};

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			result_mat[i] += endEffector[j] * y3[j][i];
		}
	}

	for (i = 0; i < 3; i++)
	{
		endEffector[i] = result_mat[i];
	}

	// Rotating Z
	result_mat = { 0.0f ,0.0f,0.0f };
	z3 = {
		{cos(angle_z), sin(angle_z), 0},
		{ -sin(angle_z), cos(angle_z), 0 },
		{ 0, 0, 1}
	};

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			result_mat[i] += endEffector[j] * z3[j][i];
		}
	}

	for (i = 0; i < 3; i++)
	{
		endEffector[i] = result_mat[i];
	}

	result_mat = { 0.0f ,0.0f,0.0f };
	// Rotating X
	x3 = {
			{1, 0, 0},
			{ 0, cos(angle_x), sin(angle_x)},
			{ 0, -sin(angle_x), cos(angle_x)}
	};

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			result_mat[i] += endEffector[j] * x3[j][i];
		}
	}

	for (i = 0; i < 3; i++)
	{
		endEffector[i] = result_mat[i];
	}

	endEffector[0] = endEffector[0] + end_effector2[0];
	endEffector[1] = endEffector[1] + end_effector2[1] ;
	endEffector[2] = endEffector[2] + end_effector2[2];

	return endEffector;
}

/*****************************************************************
* Determines the Jacobian Matrix for BONE 1.
*****************************************************************/
glm::vec3 Bone_Animation::jacobian_bone_1(glm::vec3 target_position, glm::vec3 end_effector, glm::vec3 angle_bone_1)
{

	glm::vec3 delta_e1 = beta *(target_position - end_effector);
	glm::vec3 e_minus_r1 = end_effector1 - joint_root;
	glm::mat3 rotationAxis1 = CalculateAxisRotation(root_bone_mat, angle_bone_1);

	//bone 1
	glm::vec3 crossProduct1 = glm::cross(glm::vec3(rotationAxis1[0][0], rotationAxis1[0][1], rotationAxis1[0][2]), e_minus_r1);
	glm::vec3 crossProduct2 = glm::cross(glm::vec3(rotationAxis1[1][0], rotationAxis1[1][1], rotationAxis1[1][2]), e_minus_r1);
	glm::vec3 crossProduct3 = glm::cross(glm::vec3(rotationAxis1[2][0], rotationAxis1[2][1], rotationAxis1[2][2]), e_minus_r1);

	//Jacob Matrix
	glm::mat3 jacob_1 = {
		{crossProduct1[0],crossProduct2[0],crossProduct3[0]},
		{crossProduct1[1],crossProduct2[1],crossProduct3[1]},
		{crossProduct1[2],crossProduct2[2],crossProduct3[2]}
	};

	//Transpose of Jacob Matrix
	glm::mat3 jacob_transpose_1 = glm::transpose(jacob_1);

	float alpha1 = getAlpha(jacob_1, jacob_transpose_1, delta_e1);
	glm::vec3 phy1 = getDeltaAngle(alpha1, jacob_transpose_1, delta_e1);

	return phy1;
}

/*****************************************************************
* Determines the Jacobian Matrix for BONE 2.
*****************************************************************/
glm::vec3 Bone_Animation::jacobian_bone_2(glm::vec3 end_effector1, glm::vec3 end_effector2, glm::vec3 angle_bone_2) {

	glm::vec3 delta_e2 = beta *(target_position - end_effector2);
	glm::vec3 e_minus_r2 = end_effector2 - end_effector1;
	glm::mat3 rotationAxis2 = CalculateAxisRotation(bone_1_mat, angle_bone_2);

	//bone 2
	glm::vec3 crossProduct4 = glm::cross(glm::vec3(rotationAxis2[0][0], rotationAxis2[0][1], rotationAxis2[0][2]), e_minus_r2);
	glm::vec3 crossProduct5 = glm::cross(glm::vec3(rotationAxis2[1][0], rotationAxis2[1][1], rotationAxis2[1][2]), e_minus_r2);
	glm::vec3 crossProduct6 = glm::cross(glm::vec3(rotationAxis2[2][0], rotationAxis2[2][1], rotationAxis2[2][2]), e_minus_r2);

	glm::mat3 jacob_2 = {
		{crossProduct4[0],crossProduct5[0],crossProduct6[0]},
		{crossProduct4[1],crossProduct5[1],crossProduct6[1]},
		{crossProduct4[2],crossProduct5[2],crossProduct6[2]}
	};

	glm::mat3 jacob_transpose_2 = glm::transpose(jacob_2);
	float alpha2 = getAlpha(jacob_2, jacob_transpose_2, delta_e2);
	glm::vec3 phy2 = getDeltaAngle(alpha2, jacob_transpose_2, delta_e2);

	return phy2;
}

/*****************************************************************
* Determines the Jacobian Matrix for BONE 3.
*****************************************************************/
glm::vec3 Bone_Animation::jacobian_bone_3(glm::vec3 end_effector2, glm::vec3 end_effector3, glm::vec3 angle_bone_3) {


	glm::vec3 delta_e3 = beta *(target_position - end_effector3);
	glm::vec3 e_minus_r3 = end_effector3 - end_effector2;
	glm::mat3 rotationAxis3 = CalculateAxisRotation(bone_3_mat, angle_bone_3);

	
	//bone 3
	glm::vec3 crossProduct7 = glm::cross(glm::vec3(rotationAxis3[0][0], rotationAxis3[0][1], rotationAxis3[0][2]), e_minus_r3);
	glm::vec3 crossProduct8 = glm::cross(glm::vec3(rotationAxis3[1][0], rotationAxis3[1][1], rotationAxis3[1][2]), e_minus_r3);
	glm::vec3 crossProduct9 = glm::cross(glm::vec3(rotationAxis3[2][0], rotationAxis3[2][1], rotationAxis3[2][2]), e_minus_r3);


	glm::mat3 jacob_3 = {
		{crossProduct7[0],crossProduct8[0],crossProduct9[0]},
		{crossProduct7[1],crossProduct8[1],crossProduct9[1]},
		{crossProduct7[2],crossProduct8[2],crossProduct9[2]}
	};

	glm::mat3 jacob_transpose_3 = glm::transpose(jacob_3);
	float alpha3 = getAlpha(jacob_3, jacob_transpose_3, delta_e3);
	glm::vec3 phy3 = getDeltaAngle(alpha3, jacob_transpose_3, delta_e3);

	return phy3;
}

/*****************************************************************
* Determines the value of step-wise; alpha.
*****************************************************************/
float Bone_Animation::getAlpha(glm::mat3 jacobMat, glm::mat3 jacobTranspose, glm::vec3 delta_e)
{
	glm::mat3 result_mat;
	float alpha;
	float numer[3];
	float denom[3];

	//Calculating the numerator
	for (j = 0; j < 3; j++)
	{
		numer[j] = 0.0f;
		for (k = 0; k < 3; k++)
			numer[j] = numer[j] + jacobTranspose[j][k] * delta_e[k];
	}
	float numerator = numer[0] * numer[0] + numer[1] * numer[1] + numer[2] * numer[2];

	//Calculating the denominator
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			result_mat[i][j] = 0;
			for (k = 0; k < 3; k++)
				result_mat[i][j] += jacobMat[i][k] * jacobTranspose[k][j];
		}
	}

	for (j = 0; j < 3; j++)
	{
		denom[j] = 0.0f;
		for (k = 0; k < 3; k++)
		{
			denom[j] = denom[j] + result_mat[j][k] * delta_e[k]; //time spend
		}
	}

	float denominator = denom[0] * denom[0] + denom[1] * denom[1] + denom[2] * denom[2];
	alpha = numerator / denominator;

	return alpha;
}

/*******************************************************************
* Determines the fraction of angle with which the bones shall move.
********************************************************************/
glm::vec3 Bone_Animation::getDeltaAngle(float alpha, glm::mat3 jacobTranspose, glm::vec3 target) {

	glm::vec3 deltaAngle;

	for (j = 0; j < 3; j++)
	{
		for (k = 0; k < 3; k++)
			deltaAngle[j] = deltaAngle[j] + jacobTranspose[j][k] * target[k];
	}
	deltaAngle = deltaAngle * alpha;

	return deltaAngle;
}


/*****************************************************************
* This functions rotates all bones based on the delta angle,
* and the Jacobian Matrix.
*****************************************************************/
float distance1 = 0.0f;
float distance2 = 0.0f;
float distance3 = 0.0f;
glm::mat4 Bone_Animation::JacobianRotation1(glm::vec3 goal, glm::vec3 endEffector, glm::vec3 angle_bone_1)
{
	float constant = 0.0001;
	glm::vec3 vec_goal = goal;
	end_effector1 = endEffector;

	glm::vec3 vec_bone_1 = angle_bone_1;

	distance1 = glm::distance(vec_goal, end_effector1);
	
	if (distance1 > 4.00000f)
	{
		float radius = glm::distance(endEffector, glm::vec3(2.0f, 1.0f, 2.0f));
		glm::vec3 phy = jacobian_bone_1(vec_goal, end_effector1, vec_bone_1);
		vec_bone_1 = vec_bone_1 + phy;
		total_bone_1 = total_bone_1 + phy;
		total_bone_2 = total_bone_2 + phy;
		total_bone_3 = total_bone_3 + phy;
		bone_1_mat = rotate_1(bone_1_mat, vec_bone_1);
		bone_2_mat = rotate_1(bone_2_mat, vec_bone_1);
		bone_3_mat = rotate_1(bone_3_mat, vec_bone_1);

		end_effector1 = rotateEndEffector_1(end_effector1, root_bone_mat, vec_bone_1);
		end_effector2 = rotateEndEffector_1(end_effector2, root_bone_mat, vec_bone_1);
		end_effector3 = rotateEndEffector_1(end_effector3, root_bone_mat, vec_bone_1);

		return bone_1_mat;
	}

	else
	{
		return bone_1_mat;
	}

}

/*****************************************************************
* This functions rotates BONES 2 & 3 based on the delta angle,
* and the Jacobian Matrix.
*****************************************************************/
glm::mat4 Bone_Animation::JacobianRotation2(glm::vec3 endEffector1, glm::vec3 endEffector2, glm::vec3 rotationaAngle)
{
	float constant = 0.0001;
	end_effector1 = endEffector1;
	end_effector2 = endEffector2;
	glm::vec3 vec_rotationAngle = rotationaAngle;

	distance2 = glm::distance(target_position, end_effector2);

	if (distance2 >= 2.003200f)
	{
		glm::vec3 phy = jacobian_bone_2(end_effector1, end_effector2, vec_rotationAngle);
		vec_rotationAngle = vec_rotationAngle + phy;
		total_bone_2 = total_bone_2 + phy;
		total_bone_3 = total_bone_3 + phy;
		bone_2_mat = rotate_2(bone_2_mat, vec_rotationAngle);
		bone_3_mat = rotate_2(bone_3_mat, vec_rotationAngle);
		end_effector2 = rotateEndEffector_2(end_effector2, bone_1_mat, vec_rotationAngle);
		end_effector3 = rotateEndEffector_2(end_effector3, bone_1_mat, vec_rotationAngle);

		return bone_2_mat;
	}

	else
	{
		return bone_2_mat;
	}
}

/*****************************************************************
* This functions rotates BONE 3 based on the delta angle,
* and the Jacobian Matrix.
*****************************************************************/
glm::mat4 Bone_Animation::JacobianRotation3(glm::vec3 endEffector2, glm::vec3 endEffector3, glm::vec3 rotationaAngle)
{
	float constant = 0.0001;
	end_effector2 = endEffector2;
	end_effector3 = endEffector3;
	glm::vec3 vec_rotationAngle = rotationaAngle;
	Sleep(3);
	distance3 = ( glm::distance(target_position, end_effector3));
	if (distance3 > 0.000001f)
	{
		glm::vec3 phy = jacobian_bone_3(end_effector2, end_effector3, vec_rotationAngle);
		vec_rotationAngle = vec_rotationAngle + phy;
		total_bone_3 = total_bone_3 + phy;

		bone_3_mat = rotate_3(bone_3_mat, vec_rotationAngle);
		end_effector3 = rotateEndEffector_3(end_effector3, bone_2_mat, vec_rotationAngle);

		return bone_3_mat;
	}

	else
	{
		return bone_3_mat;
	}
}

/********************************************************
* This functions calculates the value of axis rotations
* in world co-ordinate system.
*********************************************************/
glm::mat3 Bone_Animation::CalculateAxisRotation(glm::mat4 parent_matrix, glm::vec3 rotationAngle)
{
	float angle_x = rotationAngle.x;
	float angle_y = rotationAngle.y;
	float angle_z = rotationAngle.z;

	float axis_x[4];
	float axis_y[4];
	float axis_z[4];

	float vec_x[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
	float vec_y[4] = { 0.0f, 1.0f, 0.0f, 0.0f };
	float vec_z[4] = { 0.0f, 0.0f, 1.0f, 0.0f };

	glm::mat4 result_mat_1;
	glm::mat4 result_mat_2;
	glm::mat4 result_mat_3;

	x = {
		{1, 0, 0, 0},
		{ 0, cos(angle_x), sin(angle_x), 0 },
		{ 0, -sin(angle_x), cos(angle_x),0},
		{0, 0, 0, 1}
	};

	z = {
	{cos(angle_z), sin(angle_z), 0, 0},
	{ -sin(angle_z), cos(angle_z), 0, 0 },
	{ 0, 0, 1, 0},
	{0, 0, 0, 1}
	};

	// ay = w-parent . rotMat(X). rotMat(z).	[0 1 0 0]T
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_mat_1[i][j] = 0;
			for (k = 0; k < 4; k++)
				result_mat_1[i][j] += parent_matrix[i][k] * x[k][j];
		}
	}

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			for (k = 0; k < 4; k++)
				result_mat_1[i][j] += result_mat_1[i][k] * z[k][j];
		}
	}

	for (j = 0; j < 4; j++)
	{
		axis_y[j] = 0.0f;
		for (k = 0; k < 4; k++)
		{
			axis_y[j] = axis_y[j] + result_mat_1[j][k] * vec_y[k];
		}
	}

	//az = w-parent . rotMat(x).[0 0 1 0]T
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_mat_1[i][j] = 0;
			for (k = 0; k < 4; k++)
				result_mat_2[i][j] += parent_matrix[i][k] * x[k][j];
		}
	}

	for (j = 0; j < 4; j++)
	{
		axis_z[j] = 0.0f;
		for (k = 0; k < 4; k++)
		{
			axis_z[j] = axis_z[j] + result_mat_2[j][k] * vec_z[k];
		}
	}

	//ax = w-parent . [1,0,0,0]T
	for (j = 0; j < 4; j++)
	{
		axis_x[j] = 0.0f;
		for (k = 0; k < 4; k++)
		{
			axis_x[j] = axis_x[j] + result_mat_3[j][k] * vec_x[k];
		}
	}

	glm::mat3 matrixDofs;
	for (i = 0; i < 3; i++)
	{
		matrixDofs[0][i] = axis_x[i];
		matrixDofs[1][i] = axis_y[i];
		matrixDofs[2][i] = axis_z[i];

	}

	return matrixDofs;
}


/*****************************************************************
* Reset function sets the position and orientation of all bones
* to the initial position.
*
* Resets the positions and orientations of the end effectors.
*****************************************************************/
void Bone_Animation::reset()
{
	//end effectors
	end_effector1 = { 2.0f, 5.0f, 2.0f };
	end_effector2 = { 2.0f, 8.0f, 2.0f };
	end_effector3 = { 2.0f, 10.0f, 2.0f };

	//draw target
	target_mat = glm::mat4(1.0f);
	target_mat = glm::translate(target_mat, target_position);

	//draw root bone
	root_bone_mat = glm::mat4(1.0f);
	root_bone_mat = glm::translate(root_bone_mat, glm::vec3(0, 0.5f, 0));
	root_bone_mat = glm::translate(root_bone_mat, root_position);

	//draw bone 1
	bone_1_mat = glm::mat4(1.0f);
	bone_1_mat = glm::scale(bone_1_mat, glm::vec3(scale_vector[1]));
	bone_1_mat = glm::translate(bone_1_mat, glm::vec3(root_position[0] / scale_vector[1].x, ((scale_vector[0].y + 0.5*(scale_vector[1].y)) / scale_vector[1].y), root_position[2] / scale_vector[1].z));

	//draw bone 2
	bone_2_mat = glm::mat4(1.0f);
	bone_2_mat = glm::scale(bone_2_mat, glm::vec3(scale_vector[2]));
	bone_2_mat = glm::translate(bone_2_mat, glm::vec3(root_position[0] / scale_vector[2].x, ((scale_vector[0].y + scale_vector[1].y + 0.5*(scale_vector[2].y)) / scale_vector[2].y), root_position[2] / scale_vector[2].z));

	//draw bone 3
	bone_3_mat = glm::mat4(1.0f);
	bone_3_mat = glm::scale(bone_3_mat, glm::vec3(scale_vector[3]));
	bone_3_mat = glm::translate(bone_3_mat, glm::vec3(root_position[0] / scale_vector[3].x, ((scale_vector[0].y + scale_vector[1].y + scale_vector[2].y + 0.5*(scale_vector[3].y)) / scale_vector[3].y), root_position[2] / scale_vector[3].z));


	//setting them up at z1=30, z2=30, z3=30;
	bone_3_mat = rotate_3(bone_3_mat, { 0.0f, 0.0f, 30.0f });
	end_effector3 = rotateEndEffector_3(end_effector3, bone_2_mat, { 0.0f, 0.0f, 30.0f });

	bone_2_mat = rotate_2(bone_2_mat, { 0.0f, 0.0f, 30.0f });
	bone_3_mat = rotate_2(bone_3_mat, { 0.0f, 0.0f, 30.0f });
	end_effector2 = rotateEndEffector_2(end_effector2, bone_1_mat, { 0.0f, 0.0f, 30.0f });
	end_effector3 = rotateEndEffector_2(end_effector3, bone_1_mat, { 0.0f, 0.0f, 30.0f });

	bone_1_mat = rotate_1(bone_1_mat, { 0.0f, 0.0f, 30.0f });
	bone_2_mat = rotate_1(bone_2_mat, { 0.0f, 0.0f,30.0f });
	bone_3_mat = rotate_1(bone_3_mat, { 0.0f, 0.0f, 30.0f });
	end_effector1 = rotateEndEffector_1(end_effector1, root_bone_mat, { 0.0f, 0.0f, 30.0f });
	end_effector2 = rotateEndEffector_1(end_effector2, root_bone_mat, { 0.0f, 0.0f, 30.0f });
	end_effector3 = rotateEndEffector_1(end_effector3, root_bone_mat, { 0.0f, 0.0f, 30.0f });

	initial_bone_1_rotation = { 0.0f, 0.0f, 30.0f };
	initial_bone_2_rotation = { 0.0f, 0.0f, 30.0f };
	initial_bone_3_rotation = { 0.0f, 0.0f, 30.0f };

	initial_end_effector1 = end_effector1;
	initial_end_effector2 = end_effector2;
	initial_end_effector3 = end_effector3;
}

