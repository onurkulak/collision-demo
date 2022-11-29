#ifndef CONTROLS_HPP
#define CONTROLS_HPP

void computeMatricesFromInputs();
void setCamera(vec3 p, vec3 d);
void setControlsWidthHeight(int w, int h);
glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();

#endif