#version 330 core

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexNormal_modelspace;
layout(location = 2) in float colliding_info;

out vec3 Normal_cameraspace;
out vec3 EyeDirection_cameraspace;
out float triangleID;
out vec3 o_diffuseColor;
out vec3 o_specularColor;
out float colliding;
// Values that stay constant for the whole mesh.
uniform mat4 MVP;
uniform mat4 MV;
uniform vec3 diffuseColor;
uniform vec3 specularColor;

void main(){


	// Output position of the vertex, in clip space : MVP * position
	gl_Position =  MVP * vec4(vertexPosition_modelspace,1);
	
	Normal_cameraspace = ( MV * vec4(vertexNormal_modelspace,0)).xyz;
	vec3 vertexPosition_cameraspace = ( MV * vec4(vertexPosition_modelspace,1)).xyz;
	EyeDirection_cameraspace = vec3(0,0,0) - vertexPosition_cameraspace;
	// gl_Position.z = length(EyeDirection_cameraspace);
	o_diffuseColor = diffuseColor;
	o_specularColor = specularColor;
	colliding = colliding_info;
}

