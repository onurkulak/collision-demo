#version 330 core


in vec3 Sphere_Position_worldspace;
in vec3 Surface_Position_worldspace;
in vec3 Sphere2SurfaceDir_worldspace;
in float reflection_cos;
in vec3 surface_normal;

// Ouput data
layout(location = 0) out vec3 color;

// Values that stay constant for the whole mesh.
uniform mat4 M;
uniform vec3 LightPositions_worldspace[20];
uniform vec3 LightValues_worldspace[20];
uniform	float LightPower;
uniform int light_cnt;
uniform int light_ind;

void main(){

	
	vec3 MaterialSpecularColor = vec3(1.0,1.0,1.0) / 3;
	vec3 colorAtSphere=vec3(0.0);
	// Distance to the light
	float distance = length( LightPositions_worldspace[light_ind] - Sphere_Position_worldspace);
	vec3 LightColor = LightValues_worldspace[light_ind];

	if (reflection_cos > 0.2){
		colorAtSphere=MaterialSpecularColor * LightColor * LightPower / (distance*distance);
	}

	float surface_distance = length( Surface_Position_worldspace - Sphere_Position_worldspace);
	float surfaceCos =  clamp( dot( -Sphere2SurfaceDir_worldspace,surface_normal ), 0,1 );

	
	color = colorAtSphere * surfaceCos ;
}