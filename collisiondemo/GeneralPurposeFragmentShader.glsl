#version 330 core

in vec3 Normal_cameraspace;
in vec3 EyeDirection_cameraspace;
in float colliding;
in vec3 o_diffuseColor;
in vec3 o_specularColor;
// Ouput data
out vec3 color;

void main(){

	
	
		vec3 n = normalize( Normal_cameraspace );
		vec3 l = normalize(EyeDirection_cameraspace );
		float cosTheta = clamp( dot( n,l ), 0,1 );
		
		color = 
			// Diffuse : "color" of the object
			o_diffuseColor +
			// Specular : reflective highlight, like a mirror
			o_specularColor * pow(cosTheta,5);
		if(colliding>0.5){
			color = color/5;
		}

}