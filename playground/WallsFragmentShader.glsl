#version 330 core

in vec3 Normal_cameraspace;
in vec3 EyeDirection_cameraspace;
in vec2 UV;
in vec2 UVPhoto;
in vec3 LightDirection_cameraspace[20];
in vec3 Surface_Position_worldspace;

// Ouput data
out vec3 color;

// Values that stay constant for the whole mesh.
uniform int light_cnt;
uniform sampler2D renderedTextures[20];
uniform sampler2D photoSampler;
uniform vec3 LightPositions_worldspace[20];
uniform vec3 LightValues_worldspace[20];
uniform	float LightPower;

void main(){
	color = vec3(0.0,0.0,0.0);

	for(int i=0;i<light_cnt;++i){
		color += texture(renderedTextures[i], UV).xyz;
	}
	/*if(UVPhoto[0] >=0 && UVPhoto[1] >= 0){
		color += texture(photoSampler, vec2(UVPhoto[0],-UVPhoto[1])).xyz;
	}*/
	
	float surfaceCos =  clamp( dot( normalize(EyeDirection_cameraspace),Normal_cameraspace ), 0,1 );
	// float d = length(EyeDirection_cameraspace);
	
	color = color * surfaceCos;

	
	
	// Material properties
	vec3 MaterialDiffuseColor = vec3(1.0);
	vec3 MaterialAmbientColor = vec3(0.1,0.1,0.1) * MaterialDiffuseColor;
	vec3 MaterialSpecularColor = vec3(0.3,0.3,0.3);

	color += 
		// Ambient : simulates indirect lighting
		MaterialAmbientColor;

	for(int i=0;i<light_cnt;++i){
		// Distance to the light
		float distance = length( LightPositions_worldspace[i] - Surface_Position_worldspace );
		vec3 LightColor = LightValues_worldspace[i];

		// Normal of the computed fragment, in camera space
		vec3 n = normalize( Normal_cameraspace );
		// Direction of the light (from the fragment to the light)
		vec3 l = normalize( LightDirection_cameraspace[i] );
		// Cosine of the angle between the normal and the light direction, 
		// clamped above 0
		//  - light is at the vertical of the triangle -> 1
		//  - light is perpendicular to the triangle -> 0
		//  - light is behind the triangle -> 0
		float cosTheta = clamp( dot( n,l ), 0,1 );
		
		// Eye vector (towards the camera)
		vec3 E = normalize(EyeDirection_cameraspace);
		// Direction in which the triangle reflects the light
		vec3 R = reflect(-l,n);
		// Cosine of the angle between the Eye vector and the Reflect vector,
		// clamped to 0
		//  - Looking into the reflection -> 1
		//  - Looking elsewhere -> < 1
		float cosAlpha = clamp( dot( E,R ), 0,1 );
		
		color += 
			// Diffuse : "color" of the object
			MaterialDiffuseColor * LightColor * LightPower * cosTheta / (distance*distance) +
			// Specular : reflective highlight, like a mirror
			MaterialSpecularColor * LightColor * LightPower * pow(cosAlpha,5) / (distance*distance);
	}

}