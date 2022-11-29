#version 330 core
#define M_PI 3.1415926535897932384626433832795

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexNormal_modelspace;

out vec3 Sphere_Position_worldspace;
out vec3 Surface_Position_worldspace;
out vec3 Sphere2SurfaceDir_worldspace;
out float reflection_cos;
out vec3 surface_normal;

// Values that stay constant for the whole mesh.
uniform mat4 M;
uniform vec3 LightPositions_worldspace[20];
uniform int light_cnt;
uniform int light_ind;

void main(){
	
	// Position of the vertex, in worldspace : M * position
	Sphere_Position_worldspace = (M * vec4(vertexPosition_modelspace,1)).xyz;
	vec3 Sphere_Normal_WorldSpace = normalize( (M * vec4(vertexNormal_modelspace,0)).xyz);

	vec3 LightPosition_ws = LightPositions_worldspace[light_ind];
	vec3 LightDirection_ws = normalize(LightPosition_ws-Sphere_Position_worldspace);
	
	Sphere2SurfaceDir_worldspace = reflect(-LightDirection_ws,Sphere_Normal_WorldSpace);

	reflection_cos = dot(LightDirection_ws,Sphere_Normal_WorldSpace);

	vec3 unitvecs[6] = vec3[6]
						(vec3(1,0,0),
						vec3(-1,0,0),
						vec3(0,1,0),
						vec3(0,-1,0),
						vec3(0,0,1),
						vec3(0,0,-1));
	float t = 1000;
	for(int i = 0; i < 6; i++){
		int dimension = i/2;
		float wallSection = 20*unitvecs[i][dimension];
		if (Sphere2SurfaceDir_worldspace[dimension] != 0.0){
			float m_t = (wallSection - Sphere_Position_worldspace[dimension]) / Sphere2SurfaceDir_worldspace[dimension];
			if(m_t > 0 && m_t < t){
				t = m_t;
				surface_normal = -unitvecs[i];
			}
		}
	}

	Surface_Position_worldspace = t*Sphere2SurfaceDir_worldspace+Sphere_Position_worldspace;

	vec3 normalized_position = normalize(Surface_Position_worldspace);
	float ycomp = asin(normalized_position.y) / (M_PI/2);
	vec2 ground_vec_n = normalize(vec2(normalized_position.x, normalized_position.z));
	float groundcomp = acos(ground_vec_n.y);
	if(ground_vec_n.x < 0){
		groundcomp = 2*M_PI - groundcomp;
	}
	groundcomp = groundcomp/(M_PI) - 1.0;
	gl_Position = vec4(groundcomp, ycomp, 0.0, 1.0);
}

