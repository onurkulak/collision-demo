#version 330 core
#define M_PI 3.1415926535897932384626433832795

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexNormal_modelspace;

out vec3 Normal_cameraspace;
out vec3 EyeDirection_cameraspace;
out vec2 UV;
out vec2 UVPhoto;
out vec3 LightDirection_cameraspace[20];
out vec3 Surface_Position_worldspace;

// Values that stay constant for the whole mesh.
uniform mat4 MVP;
uniform mat4 V;
uniform mat4 M;
uniform int light_cnt;
uniform vec3 LightPositions_worldspace[20];

void main(){


	// Output position of the vertex, in clip space : MVP * position
	gl_Position =  MVP * vec4(vertexPosition_modelspace,1);
	
	// Vector that goes from the vertex to the camera, in camera space.
	// In camera space, the camera is at the origin (0,0,0).
	vec3 vertexPosition_cameraspace = ( V * M * vec4(vertexPosition_modelspace,1)).xyz;
	EyeDirection_cameraspace = vec3(0,0,0) - vertexPosition_cameraspace;
	
	// Normal of the the vertex, in camera space
	Normal_cameraspace = ( V * M * vec4(vertexNormal_modelspace,0)).xyz; // Only correct if ModelMatrix does not scale the model ! Use its inverse transpose if not.
	
	
	Surface_Position_worldspace = (M * vec4(vertexPosition_modelspace,1)).xyz;

	vec3 normalized_position = normalize(Surface_Position_worldspace);
	float ycomp = (asin(normalized_position.y)  + M_PI/2 ) / (M_PI);
	vec2 ground_vec_n = normalize(vec2(normalized_position.x, normalized_position.z));
	float groundcomp = acos(ground_vec_n.y);
	if(ground_vec_n.x < 0){
		groundcomp = 2*M_PI - groundcomp;
	}
	groundcomp = groundcomp/(2*M_PI);
	UV = vec2(groundcomp, ycomp);

	UVPhoto = vec2(-1,-1);
	if(vertexNormal_modelspace == vec3(-1.0,0.0,0.0)){
		UVPhoto = vertexPosition_modelspace.zy / 20;
	}

	for(int i=0;i<light_cnt;++i)
	{
		// Vector that goes from the vertex to the light, in camera space. M is ommited because it's identity.
		vec3 LightPosition_cameraspace = ( V * vec4(LightPositions_worldspace[i],1)).xyz;
		LightDirection_cameraspace[i] = LightPosition_cameraspace + EyeDirection_cameraspace;
	}
}

