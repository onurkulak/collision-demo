// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <vector>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/norm.hpp>
using namespace glm;
using namespace std;

#include <iostream>
#include <common/shader.hpp>
#include <common/texture.hpp>
#include <common/controls.hpp>


// return the triangularized vertices for a unit sphere
vector<glm::vec3> getSphereVertices(vector<glm::vec3> & normals){
	// granularity of our triangles
	float approximateHeigthOfTriangle = 0.1;
	float radius= 3;

	vector<glm::vec3> quadVertices;
	
	int quadCnt = cos(approximateHeigthOfTriangle)*M_PI_2 / approximateHeigthOfTriangle;

	vector<glm::vec3> triangleVertices;

	// calculate only 1/8 of the sphere, then use symmetry to generate other points
	for(int i=0; i<quadCnt; i++){
		float upperyangle = approximateHeigthOfTriangle*(i+1);
		float bottomyangle = upperyangle-approximateHeigthOfTriangle;
		int quadCntForTheBand = quadCnt-i;

		float baseAngleGranularity = M_PI_2 / quadCntForTheBand;
		float y1 = sin(upperyangle);
		float y2 = sin(bottomyangle);
		float baseLen1 = cos(upperyangle);
		float baseLen2 = cos(bottomyangle);

		// we are gonna duplicate some points but it's fine
		for (int quadCntr = 0; quadCntr < quadCntForTheBand; quadCntr++)
		{
			float baseFirstAngle = quadCntr * baseAngleGranularity;
			float baseSecondAngle = baseFirstAngle + baseAngleGranularity;
			float x1 = baseLen1 * cos(baseFirstAngle);
			float x2 = baseLen2 * cos(baseSecondAngle);
			float z1 = baseLen1 * sin(baseFirstAngle);
			float z2 = baseLen2 * sin(baseSecondAngle);

			float x3 = baseLen1 * cos(baseSecondAngle);
			float x4 = baseLen2 * cos(baseFirstAngle);
			float z3 = baseLen1 * sin(baseSecondAngle);
			float z4 = baseLen2 * sin(baseFirstAngle);
			quadVertices.push_back(glm::vec3(x1, y1, z1));
			if(i>0){
				quadVertices.push_back(quadVertices[quadVertices.size()+1-(quadCntForTheBand+1-quadCntr*2/quadCntForTheBand)*4]);
			}
			else
			{
				quadVertices.push_back(glm::vec3(x2, y2, z2));	
			}
			
			quadVertices.push_back(glm::vec3(x3, y1, z3));
			if(i>0){
				quadVertices.push_back(quadVertices[quadVertices.size()-3-(quadCntForTheBand+1-quadCntr*2/quadCntForTheBand)*4]);
			}
			else
			{
				quadVertices.push_back(glm::vec3(x4, y2, z4));
			}

			// add the triangle because we are just passing through it
			if(i>0 && quadCntr*2>=quadCntForTheBand && (quadCntr-1)*2<quadCntForTheBand){

				vec3 tp[3] = {quadVertices[quadVertices.size()-4], quadVertices[quadVertices.size()-7], quadVertices[quadVertices.size()-1]};

				triangleVertices.push_back(tp[0]*radius);
				triangleVertices.push_back(tp[2]*radius);
				triangleVertices.push_back(tp[1]*radius);
				normals.push_back(cross( tp[2] - tp[0], tp[1] - tp[0] ));
				normals.push_back(cross( tp[2] - tp[0], tp[1] - tp[0] ));
				normals.push_back(cross( tp[2] - tp[0], tp[1] - tp[0] ));
			}
		}
	}
	// add the final triangle at the top
	vec3 tp[3] = {vec3(0.0f,1.0f,0.0f), quadVertices[quadVertices.size()-4], quadVertices[quadVertices.size()-2]};
	triangleVertices.push_back(tp[0] * radius);
	triangleVertices.push_back(tp[2] * radius);
	triangleVertices.push_back(tp[1] * radius);
	normals.push_back(cross(tp[2] - tp[0], tp[1] - tp[0]));
	normals.push_back(cross(tp[2] - tp[0], tp[1] - tp[0]));
	normals.push_back(cross(tp[2] - tp[0], tp[1] - tp[0]));

	
	vec3 tp2[3] = {quadVertices[quadVertices.size()-2], quadVertices[quadVertices.size()-3], quadVertices[quadVertices.size()-6]};
	triangleVertices.push_back(tp2[0] * radius);
	triangleVertices.push_back(tp2[2] * radius);
	triangleVertices.push_back(tp2[1] * radius);
	normals.push_back(cross(tp2[2] - tp2[0], tp2[1] - tp2[0]));
	normals.push_back(cross(tp2[2] - tp2[0], tp2[1] - tp2[0]));
	normals.push_back(cross(tp2[2] - tp2[0], tp2[1] - tp2[0]));

	for(int i = 0; i < quadVertices.size(); i++){
		quadVertices[i] *= radius;
	}

	cout<< quadVertices.size() << endl;

	for (int i = 0; i < quadVertices.size(); i+=4){
		triangleVertices.push_back(glm::vec3(quadVertices[i]));
		triangleVertices.push_back(glm::vec3(quadVertices[i+2]));
		triangleVertices.push_back(glm::vec3(quadVertices[i+1]));
		triangleVertices.push_back(glm::vec3(quadVertices[i+1]));
		triangleVertices.push_back(glm::vec3(quadVertices[i+3]));
		triangleVertices.push_back(glm::vec3(quadVertices[i]));
		// for every point, add the quad normal as the normal
		glm::vec3 normal(0.0f, 0.0f, 0.0f);
		for (int j = 0; j < 4 ; j++){
			normal += quadVertices[i+j];
		}
		normal = glm::normalize(normal);
		for (int j=0; j<6; j++){
			normals.push_back(normal);
		}
	}
	cout<< triangleVertices.size() << endl;

	int originalVerticesSize = triangleVertices.size();
	// take the symmetries fo the existing triangle vertices
	for (int i = 1; i < 8; i++){
		glm::vec3 mask(
			(i & 1) ? -1.0f : 1.0f,
			(i & 2) ? -1.0f : 1.0f,
			(i & 4) ? -1.0f : 1.0f
		);
		bool oddtransform = bool(i & 1) != bool(i & 2) != bool(i & 4);
		// fix the normals for some of the points by reordering every 2nd and 3rd point
		if(oddtransform){
			for (int j = 0; j < originalVerticesSize; j+=3){
				triangleVertices.push_back(triangleVertices[j]*mask);
				triangleVertices.push_back(triangleVertices[j+2]*mask);
				triangleVertices.push_back(triangleVertices[j+1]*mask);
			}
		}
		else{
			for (int j = 0; j < originalVerticesSize; j++){
				triangleVertices.push_back(triangleVertices[j]*mask);
			}
		}
		// normals our same
		for (int j = 0; j < originalVerticesSize; j++){
			normals.push_back(normals[j]*mask);
		}
		
	}
	cout<< triangleVertices.size() << endl;
	return triangleVertices;
}



// return the triangularized vertices for a unit cube
vector<glm::vec3> getRoomVertices(vector<glm::vec3> & normals){
	
	// granularity of our triangles
	float length = 20;

	GLfloat g_vertex_buffer_data[] = {
    -1.0f,-1.0f,-1.0f, // triangle 1 : begin
    -1.0f,-1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f, // triangle 1 : end
    1.0f, 1.0f,-1.0f, // triangle 2 : begin
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f, // triangle 2 : end
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f,
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f
	};


	vector<glm::vec3> triangleVertices;
	for(int i = 0; i < 36*3; i+=3){
		triangleVertices.push_back(glm::vec3(g_vertex_buffer_data[i], g_vertex_buffer_data[i+1], g_vertex_buffer_data[i+2]));
	}
	
	for (int i = 0; i < triangleVertices.size(); i+=3){
		
		glm::vec3 normal(0.0f, 0.0f, 0.0f);
		for (int j = 0; j < 3 ; j++){
			normal += triangleVertices[i+j];
			triangleVertices[i+j] *= length;
		}

		for (int j = 0; j < 3; j++){
			if( abs(normal [j]) > 2.5f){
				normal[j] = (normal[j] < 0) - (normal[j] > 0);
			}
			else{
				normal[j] = 0.0f;
			}
		}

		for (int j=0; j<3; j++){
			normals.push_back(normal);
		}
		// cout << normal.x << " " << normal.y << " " << normal.z << endl;

		if (glm::dot(normal, glm::cross(triangleVertices[i+1] - triangleVertices[i],
							triangleVertices[i+1] - triangleVertices[i+2])) > 0){
								swap(triangleVertices[i+1], triangleVertices[i+2]);
							}
	}
	
	return triangleVertices;
}


int main( void )
{
	// Initialise GLFW
	if( !glfwInit() )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    int windowWidth = 3800;
    int windowHeight = 2100;

	// Open a window and create its OpenGL context
	window = glfwCreateWindow( windowWidth, windowHeight, "Tutorial 0 - Keyboard and Mouse", NULL, NULL);
	if( window == NULL ){
		fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
		getchar();
		glfwTerminate();
		return -1;
	}
    glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    // Hide the mouse and enable unlimited mouvement
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
    // Set the mouse at the center of the screen
    glfwPollEvents();
    glfwSetCursorPos(window, windowWidth/2, windowHeight/2);

	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);


	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);


	vector<glm::vec3> sphereNormals;
	vector<glm::vec3> sphereVertices = getSphereVertices(sphereNormals);

	
	vector<glm::vec3> roomNormals;
	vector<glm::vec3> roomVertices = getRoomVertices(roomNormals);


	GLuint vertexbuffers[2];
	glGenBuffers(2, vertexbuffers);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers[0]);
	glBufferData(GL_ARRAY_BUFFER, sphereVertices.size() * sizeof(glm::vec3), &sphereVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers[1]);
	glBufferData(GL_ARRAY_BUFFER, roomVertices.size() * sizeof(glm::vec3), &roomVertices[0], GL_STATIC_DRAW);

	GLuint normalbuffers[2];
	glGenBuffers(2, normalbuffers);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffers[0]);
	glBufferData(GL_ARRAY_BUFFER, sphereNormals.size() * sizeof(glm::vec3), &sphereNormals[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffers[1]);
	glBufferData(GL_ARRAY_BUFFER, roomNormals.size() * sizeof(glm::vec3), &roomNormals[0], GL_STATIC_DRAW);

	// program for creating reflections from the ball
	GLuint sphereReflectionProgram = LoadShaders( "SphereVertexShaderReflection.glsl", "SphereFragmentShaderReflection.glsl" );

	// Get a handle for our "MVP" uniform
	GLuint ModelMatrixIDRef = glGetUniformLocation(sphereReflectionProgram, "M");

	GLuint LightPOSRef = glGetUniformLocation(sphereReflectionProgram, "LightPositions_worldspace");
	GLuint LightVALRef = glGetUniformLocation(sphereReflectionProgram, "LightValues_worldspace");
	GLuint LightPOWRef = glGetUniformLocation(sphereReflectionProgram, "LightPower");
	GLuint LightCNTRef = glGetUniformLocation(sphereReflectionProgram, "light_cnt");
	GLuint LightReflectionIndexRef = glGetUniformLocation(sphereReflectionProgram, "light_ind");

	// program for illuminating the disco ball
	GLuint sphereProgram = LoadShaders( "SphereVertexShader.glsl", "SphereFragmentShader.glsl" );

	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(sphereProgram, "MVP");
	GLuint ViewMatrixID = glGetUniformLocation(sphereProgram, "V");
	GLuint ModelMatrixID = glGetUniformLocation(sphereProgram, "M");

	GLuint LightPOS = glGetUniformLocation(sphereProgram, "LightPositions_worldspace");
	GLuint LightVAL = glGetUniformLocation(sphereProgram, "LightValues_worldspace");
	GLuint LightPOW = glGetUniformLocation(sphereProgram, "LightPower");
	GLuint LightCNT = glGetUniformLocation(sphereProgram, "light_cnt");


	// apply reflections and original lights onto the room walls
	GLuint roomProgram = LoadShaders( "WallsVertexShader.glsl", "WallsFragmentShader.glsl" );

	// Get a handle for our "MVP" uniform
	GLuint MatrixIDR = glGetUniformLocation(roomProgram, "MVP");
	GLuint ViewMatrixIDR = glGetUniformLocation(roomProgram, "V");
	GLuint ModelMatrixIDR = glGetUniformLocation(roomProgram, "M");

	GLuint LightPOSR = glGetUniformLocation(roomProgram, "LightPositions_worldspace");
	GLuint LightVALR = glGetUniformLocation(roomProgram, "LightValues_worldspace");
	GLuint LightPOWR = glGetUniformLocation(roomProgram, "LightPower");
	GLuint LightCNTR = glGetUniformLocation(roomProgram, "light_cnt");

	const int light_cnt = 4;
	float a = 7.0f;
	GLfloat lightPos[light_cnt*3] = {a,a,a,
									-a,a,a,
									0,-a/2,a};
	GLfloat lightVal[light_cnt*3] = {1.0,0,0,
									0,1.0,0,
									0,0,1.0};
	GLfloat lightPower = 250.0;


	// generating bunch of buffers and textures, per light source
	
	// The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
	GLuint ReflectionFramebuffers[light_cnt];
	glGenFramebuffers(light_cnt, ReflectionFramebuffers);
	GLuint reflectionTextures[light_cnt];
	glGenTextures(light_cnt, reflectionTextures);

	// Load the photo texture
	GLuint photoTexture = loadBMP_custom("askkus.bmp");
	GLuint photoTextureID  = glGetUniformLocation(roomProgram, "photoSampler");

	GLuint texSampler = glGetUniformLocation(roomProgram, "renderedTextures");
	vector<int> texIDs;
	for(int i = 0; i < light_cnt; i++){
		texIDs.push_back(i);
	}

	vector<GLenum> DrawBuffers;

	for (int i=0; i<light_cnt; i++){
		glBindFramebuffer(GL_FRAMEBUFFER, ReflectionFramebuffers[i]);
		
		// "Bind" the newly created texture : all future texture functions will modify this texture
		glBindTexture(GL_TEXTURE_2D, reflectionTextures[i]);

		// Give an empty image to OpenGL ( the last "0" means "empty" )
		glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, windowWidth, windowHeight, 0,GL_RGB, GL_UNSIGNED_BYTE, 0);

		// Poor filtering
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		// Set "renderedTexture" as our colour attachement #0
		glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, reflectionTextures[i], 0);
		DrawBuffers.push_back(GL_COLOR_ATTACHMENT0);
	}
	
	glDrawBuffers(DrawBuffers.size(), &DrawBuffers[0]); 


	// Always check that our framebuffer is ok
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		return false;


	
	
	// The fullscreen quad's FBO
	static const GLfloat g_quad_vertex_buffer_data[] = { 
		-1.0f, -1.0f, 0.0f,
		 1.0f, -1.0f, 0.0f,
		-1.0f,  1.0f, 0.0f,
		-1.0f,  1.0f, 0.0f,
		 1.0f, -1.0f, 0.0f,
		 1.0f,  1.0f, 0.0f,
	};

	GLuint quad_vertexbuffer;
	glGenBuffers(1, &quad_vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, quad_vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), g_quad_vertex_buffer_data, GL_STATIC_DRAW);

	// Create and compile our GLSL program from the shaders
	GLuint quad_programID = LoadShaders( "DebugVertexShader.glsl", "DebugFragmentShader.glsl" );
	GLuint texID = glGetUniformLocation(quad_programID, "renderedTexture");
	


	setControlsWidthHeight(windowWidth, windowHeight);

	vec3 gOrientation1;
	do{


		// Enable depth test
		glEnable(GL_DEPTH_TEST);
		// Accept fragment if it closer to the camera than the former one
		glDepthFunc(GL_LESS); 

		// Cull triangles which normal is not towards the camera
		// removed until I add normals
		glEnable(GL_CULL_FACE);

		// Compute the MVP matrix from keyboard and mouse input
		computeMatricesFromInputs();
		glm::mat4 ProjectionMatrix = getProjectionMatrix();
		glm::mat4 ViewMatrix = getViewMatrix();
		double currentTime = glfwGetTime();
		gOrientation1.y = 3.14159f/4.0f * currentTime/5;
 
		// Build the model matrix
		glm::mat4 RotationMatrix = eulerAngleYXZ(gOrientation1.y, gOrientation1.x, gOrientation1.z);
		glm::mat4 ModelMatrix = RotationMatrix;
		glm::mat4 RoomModelMatrix = glm::mat4(1.0);


		glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
		glm::mat4 RoomMVP = ProjectionMatrix * ViewMatrix * RoomModelMatrix;



		// Render to the screen
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
        // Render on the whole framebuffer, complete from the lower left corner to the upper right
		glViewport(0,0,windowWidth,windowHeight);

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Use our shader
		glUseProgram(sphereProgram);
		// Send our transformation to the currently bound shader, 
		// in the "MVP" uniform
		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
		glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
		glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);

		glUniform1i(LightCNT, light_cnt);
		glUniform3fv(LightPOS, light_cnt, &lightPos[0]);
		glUniform3fv(LightVAL, light_cnt, &lightVal[0]);
		glUniform1f(LightPOW, lightPower);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers[0]);
		glVertexAttribPointer(
			0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);

		// 2nd attribute buffer : normals
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, normalbuffers[0]);
		glVertexAttribPointer(
			1,                                // attribute
			3,                                // size
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
		);

		// Draw the triangle !
		glDrawArrays(GL_TRIANGLES, 0, sphereVertices.size()); // draw all triangles

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);


		// glDisable(GL_DEPTH_TEST);
		//glDisable(GL_CULL_FACE);
		// Use our shader for reflections
		glUseProgram(sphereReflectionProgram);

		glUniformMatrix4fv(ModelMatrixIDRef, 1, GL_FALSE, &ModelMatrix[0][0]);

		glUniform1i(LightCNTRef, light_cnt);
		glUniform3fv(LightPOSRef, light_cnt, &lightPos[0]);
		glUniform3fv(LightVALRef, light_cnt, &lightVal[0]);
		glUniform1f(LightPOWRef, lightPower);

		// render to separate framebuffers to create all textures
		for(int i = 0; i <light_cnt; i++){
			// Render to our framebuffer
			glBindFramebuffer(GL_FRAMEBUFFER, ReflectionFramebuffers[i]);
			// Clear the screen
			glClear(GL_COLOR_BUFFER_BIT);

			glUniform1i(LightReflectionIndexRef, i);
			glViewport(0,0,windowWidth,windowHeight);
			

			// do I really need to resend all of them for every texture????
			// 1rst attribute buffer : vertices
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers[0]);
			glVertexAttribPointer(
				0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
				3,                  // size
				GL_FLOAT,           // type
				GL_FALSE,           // normalized?
				0,                  // stride
				(void*)0            // array buffer offset
			);

			// 2nd attribute buffer : normals
			glEnableVertexAttribArray(1);
			glBindBuffer(GL_ARRAY_BUFFER, normalbuffers[0]);
			glVertexAttribPointer(
				1,                                // attribute
				3,                                // size
				GL_FLOAT,                         // type
				GL_FALSE,                         // normalized?
				0,                                // stride
				(void*)0                          // array buffer offset
			);

			
			glDrawArrays(GL_TRIANGLES, 0, sphereVertices.size()); // draw all triangles

			glDisableVertexAttribArray(0);
			glDisableVertexAttribArray(1);
		}


		// Render to the screen
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
        // Render on the whole framebuffer, complete from the lower left corner to the upper right
		glViewport(0,0,windowWidth,windowHeight);


		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		// Use our shader for the room
		glUseProgram(roomProgram);

		// Send our transformation to the currently bound shader, 
		// in the "MVP" uniform
		glUniformMatrix4fv(MatrixIDR, 1, GL_FALSE, &RoomMVP[0][0]);
		glUniformMatrix4fv(ModelMatrixIDR, 1, GL_FALSE, &RoomModelMatrix[0][0]);
		glUniformMatrix4fv(ViewMatrixIDR, 1, GL_FALSE, &ViewMatrix[0][0]);

		glUniform1i(LightCNTR, light_cnt);

		for(int i=0; i<light_cnt; i++){
			// Bind our texture in Texture Unit 
			glActiveTexture(GL_TEXTURE0 + i);
			glBindTexture(GL_TEXTURE_2D, reflectionTextures[i]);
		}
		glActiveTexture(GL_TEXTURE0 + light_cnt);
		glBindTexture(GL_TEXTURE_2D, photoTexture);		

		// bind texture samplers
		glUniform1iv(texSampler, light_cnt, &texIDs[0]);
		glUniform1i(photoTextureID, light_cnt);
		glUniform3fv(LightPOSR, light_cnt, &lightPos[0]);
		glUniform3fv(LightVALR, light_cnt, &lightVal[0]);
		glUniform1f(LightPOWR, lightPower);


		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers[1]);
		glVertexAttribPointer(
			0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);

		// 2nd attribute buffer : normals
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, normalbuffers[1]);
		glVertexAttribPointer(
			1,                                // attribute
			3,                                // size
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
		);

		// Draw the triangle !
		glDrawArrays(GL_TRIANGLES, 0, roomVertices.size()); // draw all riangles

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);


		// debug code 
		// Render to the screen
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
        // Render on the whole framebuffer, complete from the lower left corner to the upper right
		glViewport(0,0,500,500);

		// Clear the screen
		// glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Use our shader
		glUseProgram(quad_programID);

		// Bind our texture in Texture Unit 0
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, reflectionTextures[0]);
		// Set our "renderedTexture" sampler to use Texture Unit 0
		glUniform1i(texID, 0);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, quad_vertexbuffer);
		glVertexAttribPointer(
			0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);

		// Draw the triangles !
		glDrawArrays(GL_TRIANGLES, 0, 6); // 2*3 indices starting at 0 -> 2 triangles

		glDisableVertexAttribArray(0);




		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

	} // Check if the ESC key was pressed or the window was closed
	while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
		   glfwWindowShouldClose(window) == 0 );

	// Cleanup VBO and shader
	glDeleteBuffers(1, vertexbuffers);
	glDeleteProgram(roomProgram);
	glDeleteProgram(sphereProgram);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return 0;
}

