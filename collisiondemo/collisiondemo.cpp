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
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/reciprocal.hpp>
#include "glm/gtx/string_cast.hpp"
#include <glm/gtc/epsilon.hpp>
#include <algorithm>    // std::sort
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace glm;
using namespace std;

#include <iostream>
#include <common/shader.hpp>
#include <common/texture.hpp>
#include <common/controls.hpp>
#include "PQP.h"

float M_EPSILON = 0.0001f;
bool verbose = false;
const int maxObjCnt = 200;
int polyhedraCnt;
int objCount;
const uint backupBufferSize=6;

float approximateLineLineIntersect(pair<vec3,vec3> l0, pair<vec3,vec3> l1, glm::vec3 & p){
	vec3 m0 = l0.first; vec3 m1 = l1.first;
	vec3 p0 = l0.second; vec3 p1 = l1.second;

	float numerator = (p1.y - p0.y) / m0.y - (p1.x - p0.x) / m0.x;
	float denominator = m1.x/m0.x - m1.y / m0.y;
	float t1 = numerator/denominator;
	
	p = t1*m1 + p1;
	if(glm::isnan(t1)){
		cout << "encountered nan value during line intersection" << endl;
		cout <<  t1 << endl;
		cout <<  glm::to_string(p) << endl; 
		cout <<  glm::to_string(m0) << endl;
		cout <<  glm::to_string(p0) << endl;
		cout <<  glm::to_string(m1) << endl;
		cout <<  glm::to_string(p1) << endl;
	}
	return t1;
}

float pointInsideTriangle(vec3 p, vector<vec3> t){
	vec3 cs[3];
	float certainty = 1.0f;
	for(int i=0; i < 3; i++){
		cs[i] = cross(t[(i+1)%3] - t[i], p - t[(i+1)%3]);
		certainty = std::min(certainty, std::min(l1Norm(t[i]-p), l1Norm(cs[i])));
	}
	bool clue = (dot ( cs[0], cs[1]) > 0);
	bool isInside = clue == (dot ( cs[0], cs[2]) > 0) && clue == (dot ( cs[1], cs[2]) > 0);
	if(isInside){
		if(verbose){
			cout << "seems to be a point inside triangle:\t" << to_string(p) << endl;
		}
		return certainty;
	}
	else{
		return 0.0f;
	}
}

float segmentFullIntersectNoOverlap(pair<vec3,vec3> l0, pair<vec3,vec3> l1, glm::vec3 & p){
	vec3 m0 = l0.first; vec3 m1 = l1.first;
	vec3 p0 = l0.second; vec3 p1 = l1.second;

	float numerator = (p1.y - p0.y) / m0.y - (p1.x - p0.x) / m0.x;
	float denominator = m1.x/m0.x - m1.y / m0.y;
	if(abs(denominator)<M_EPSILON){
		return false;
	}
	float t1 = numerator/denominator;
	float onSegmentCertainty = std::min(abs(t1), abs(t1-1.0f));
	p = t1*m1 + p1;
	float t2 = (p-p0).x / m0.x;
	onSegmentCertainty = std::min(onSegmentCertainty, std::min(abs(t2), abs(t2-1.0f)));

	bool isOnSegment = t1>0.0f && t1<1.0f && t2>0.0f && t2<1.0f;

	if(verbose){
		if(isOnSegment){
			cout << "line segment intersection" << endl;
			cout << "certainty:" << onSegmentCertainty << endl;
			cout << (p-p0).x << '\n' << m0.x << endl;
			cout << denominator<<endl;
			cout << to_string(p0) << to_string(m0) << endl;
			cout << to_string(p1) << to_string(m1) << endl;
		}
	}

	if (isOnSegment)
		return onSegmentCertainty;
	else
		return 0.0f;
}

float triangleTriangleOverlap(vector<vec3> t0, vector<vec3> t1){
	// check for line intersection
	float certainty = 0.0f;
	for(int i=0; i < 3; i++){
		pair<vec3,vec3> l0(t0[(i+1)%3]-t0[i] , t0[i]);
		for(int j=0; j < 3; j++){
			if((l1Norm(t0[i] - t1[j]) < M_EPSILON*3 && l1Norm(t0[(i+1)%3] - t1[(j+1)%3]) < M_EPSILON*3)
			|| (l1Norm(t0[i] - t1[(j+1)%3]) < M_EPSILON*3 && l1Norm(t0[(i+1)%3] - t1[j]) < M_EPSILON*3)){
				continue;
			}
			pair<vec3,vec3> l1(t1[(j+1)%3]-t1[j] , t1[j]);
			vec3 p;
			certainty = std::max(certainty, segmentFullIntersectNoOverlap(l0, l1, p));
		}	
	}
	
	float subsetCertainty = std::max ( pointInsideTriangle(t0[0], t1), pointInsideTriangle(t1[0], t0)) ;
	
	if(verbose){
		if(certainty > M_EPSILON){
		cout << "looks like a line intersection w certainty:\t" << certainty <<endl;
		}
		// check whether a triangle is completely inside the other
		if(subsetCertainty > M_EPSILON){
			cout << "point inside triangle w certainty:\t" << subsetCertainty << endl;
		}
	}
	return std::max(certainty, subsetCertainty);
}


float findAnglePartition(float totalRadian, float sinRatio){
	float x = 0.5;
	float stepSize = 0.25;
	while( abs (sin(totalRadian*x) / sin(totalRadian*(1-x)) - sinRatio) > M_EPSILON){
		if(sin(totalRadian*x) / sin(totalRadian*(1-x)) > sinRatio){
			x-=stepSize;
		}
		else{
			x+=stepSize;
		}
		stepSize/=1.5;
	}
	return x;
}

bool isTriangleDisjoint(const vector<vector<glm::vec3>> & triangles, vector<glm::vec3> triangle){
	if(triangles.size() <= 1){
		return false;
	}

	for(int s=0; s<triangles.size() ; s++){						
		int samePointCnt = 0;
		for(int j = 0; j < 3; j++){
			for(int k = 0; k < 3; k++){
				if( glm::l2Norm(triangle[j] - triangles[s][k]) < 0.00001){
					samePointCnt++;
				}
			}
		}
		if(samePointCnt == 2){
			return false;
		}
	}
	return true;
}

// calculates an approximation of the inertia tensor for a given set of vertices, 
// the object is assuemd to be a thin shell without inner volume
// it first enumerates new vertices to improve the integral using the level of detail parameter
mat3 getInertiaTensorForPolyhedra(vector<vec3>& vertices, int levelOfDetail=2, float mass=1.0f){
	float area = 0.0f;
	// for double buffering
	vector<vec3> v0;
	vector<vec3> v1;

	mat3 it(0.0f);
	
	// for each triangle, do st like the subdivision method, create bunch of new triangles inside it
	// then for each vertex in there calculate the inertia effect
	for(int i=0; i<vertices.size(); i+=3){
		float tArea = l2Norm(cross(vertices[i+1]-vertices[i], vertices[i+2]-vertices[i]));
		area += tArea;
		mat3 tit(0.0f);

		v0.clear();
		v1.clear();
		auto vCurPtr = &v0;
		auto vEnumeratePtr = &v1;
		v0.push_back(vertices[i]);
		v0.push_back(vertices[i+1]);
		v0.push_back(vertices[i+2]);
		for(int lod=0; lod<levelOfDetail; lod++){
			for(int t=0; t<vCurPtr->size(); t+=3){
				vec3 pt0 = vCurPtr->at(t);
				vec3 pt1 = vCurPtr->at(t+1);
				vec3 pt2 = vCurPtr->at(t+2);
				vec3 pt01 = (pt0+pt1)/2.0f;
				vec3 pt02 = (pt0+pt2)/2.0f;
				vec3 pt12 = (pt2+pt1)/2.0f;
				vEnumeratePtr->push_back(pt0);
				vEnumeratePtr->push_back(pt01);
				vEnumeratePtr->push_back(pt02);
				vEnumeratePtr->push_back(pt1);
				vEnumeratePtr->push_back(pt01);
				vEnumeratePtr->push_back(pt12);
				vEnumeratePtr->push_back(pt2);
				vEnumeratePtr->push_back(pt02);
				vEnumeratePtr->push_back(pt12);
				vEnumeratePtr->push_back(pt02);
				vEnumeratePtr->push_back(pt01);
				vEnumeratePtr->push_back(pt12);
			}
			vCurPtr->clear();
			swap(vCurPtr, vEnumeratePtr);
		}
		// at the end vcurptr has all the enumerated points
		for(int j=0; j<vCurPtr->size(); j++){
			// inertia tensor effect per vertex
			vec3 pt = vCurPtr->at(j);
			tit[0][0] += pt.y*pt.y + pt.z*pt.z;
			tit[2][2] += pt.y*pt.y + pt.x*pt.x;
			tit[1][1] += pt.x*pt.x + pt.z*pt.z;
			tit[0][1] -= pt.x*pt.y;
			tit[0][2] -= pt.x*pt.z;
			tit[1][2] -= pt.z*pt.y;
		}

		tit[1][0] = tit[0][1];
		tit[2][0] = tit[0][2];
		tit[2][1] = tit[1][2];
		// effect is proportional to the triangle's area
		tit*=tArea;
		it+=tit;
	}

	it *= 1.0f/area*mass;
	return it;
}

bool cutTriangles(pair<vec3,vec3> l, vector<vector<glm::vec3>> & triangles, vec3 center){
	int trianglesSize = triangles.size(); 
	for(int i = 0; i < trianglesSize; i++){
		vec3 cutPoints[3];
		int ind=0;
		vec3 closePoints[2];
		vec3 farPoint;
		vector<vec3> t = triangles[i];
		for(int j = 0; j < 3; j++){
			float segmentParam = approximateLineLineIntersect(l, pair(t[(j+1)%3]-t[j], t[j]), cutPoints[ind]);
			if(!glm::isnan(segmentParam) && segmentParam>0 && segmentParam<1.0f){
				ind++;
			}
			else{
				farPoint = t[(j+2)%3];
				closePoints[0] = t[j];
				closePoints[1] = t[(j+1)%3];
			}
		}
		// there's something wrong with our methods, give error
		if( ind%2 == 1){
			cout <<  glm::to_string(cutPoints[0]) << endl;
			cout <<  glm::to_string(cutPoints[1]) << endl;
			cout <<  glm::to_string(cutPoints[2]) << endl;
			cout <<  glm::to_string(t[0]) << endl;
			cout <<  glm::to_string(t[1]) << endl;
			cout <<  glm::to_string(t[2]) << endl;
			cout <<  glm::to_string(l.first) << endl;
			cout <<  glm::to_string(l.second) << endl;
			cout<<ind<<endl;
			return false;
		}
		vec3 centerDir = cross(l.first, center-l.second);
		vec3 farDir = cross(l.first, farPoint-l.second);
		if(ind == 0){
			// no intersection with this triangle
			// still check if it's inside the line
			
			if(dot(centerDir, farDir) < 0){
				triangles.erase(triangles.begin() + i);
				i--; trianglesSize--;
			}
		}
		else{
			// we get the smaller triangle
			if(dot(farDir, centerDir) > 0){
				vector<vec3> newt;
				newt.push_back(cutPoints[0]);
				newt.push_back(cutPoints[1]);
				newt.push_back(farPoint);
				triangles[i] = newt;
			}
			// we get the larger quad and form 2 triangles from that
			else{
				vector<vec3> newt;
				newt.push_back(cutPoints[0]);
				newt.push_back(cutPoints[1]);
				newt.push_back(closePoints[0]);
				vector<vec3> newt2Candidate0;
				newt2Candidate0.push_back(closePoints[1]);
				newt2Candidate0.push_back(closePoints[0]);
				newt2Candidate0.push_back(cutPoints[1]);
				vector<vec3> newt2Candidate1;
				newt2Candidate1.push_back(closePoints[1]);
				newt2Candidate1.push_back(closePoints[0]);
				newt2Candidate1.push_back(cutPoints[0]);
				if(triangleTriangleOverlap(newt, newt2Candidate0) > triangleTriangleOverlap(newt, newt2Candidate1)){
					triangles.push_back(newt2Candidate1);
				}
				else{
					triangles.push_back(newt2Candidate0);
				}
				triangles[i] = newt;
			}
		}
		if(verbose){
			// sanity check-debug, whether we have overlapping triangles

			for (int j = 0; j < triangles.size(); j++)
			{
				for (int k = 0; k < j; k++)
				{
					float overlapCertainty = triangleTriangleOverlap(triangles[j], triangles[k]);
					if (overlapCertainty > 0.1)
					{

						cout << "printing points" << endl;
						for (int l = 0; l < 3; l++)
						{
							cout << to_string(triangles[j][l]) << endl;
						}
						for (int l = 0; l < 3; l++)
						{
							cout << to_string(triangles[k][l]) << endl;
						}

						cout << "alert overlap detected:\n";
						cout << "certainty:\t" << overlapCertainty << endl;
						cout << j << " " << k << endl;
						cout << dot(farDir, centerDir) << endl;
						cout << "triangle points:" << endl;
						cout << to_string(farPoint) << endl;
						cout << to_string(closePoints[0]) << endl;
						cout << to_string(closePoints[1]) << endl;
						cout << "cut points:" << endl;
						cout << to_string(cutPoints[0]) << endl;
						cout << to_string(cutPoints[1]) << endl;
						return false;
					}
				}
			}
			float centerInside = 0.0f;
			for (int j = 0; j < triangles.size(); j++)
			{
				centerInside = std::max(centerInside, pointInsideTriangle(center, triangles[j]));
			}
			if (centerInside==0.0f)
			{
				cout << "lost the center somewhere:\n";
				cout << "certainty:\t" << centerInside << endl;
				cout << i << endl;
				cout << to_string(centerDir) << endl;
				cout << to_string(farDir) << endl;
				cout << dot(farDir, centerDir) << endl;
				cout << "center:\t" << to_string(center) << endl;
				cout << "triangle points:" << endl;
				cout << to_string(farPoint) << endl;
				cout << to_string(closePoints[0]) << endl;
				cout << to_string(closePoints[1]) << endl;
				cout << "cut points:" << endl;
				cout << to_string(cutPoints[0]) << endl;
				cout << to_string(cutPoints[1]) << endl;
				return false;
			}

		}
	}
	return true;
}

pair<vec3, vec3> getPlanePlaneIntersect(const vec3 & d0, const vec3 & d1){
	vec3 n0 = d0 / glm::l2Norm(d0);
	vec3 n1 = d1 / glm::l2Norm(d1);
		
	vec3 linem = glm::cross(d0, d1);
	float angleBetween = glm::pi<float>() - acos(glm::dot(n0, n1));
	float angleRatio = findAnglePartition(angleBetween, glm::l2Norm(d0) / glm::l2Norm(d1));
	float deviationLength = cos(angleRatio*angleBetween) / sin(angleRatio*angleBetween) * glm::l2Norm(d0);
			
	vec3 deviationDir = dot(n0,n1) * -n0 + n1;
	vec3 deviation = deviationDir * (deviationLength / glm::l2Norm(deviationDir));
	vec3 pointOnLine = deviation + d0;
	return pair<vec3,vec3>(linem, pointOnLine);
}


vector<glm::vec3> findFundamentalTriangle(int *cutterIndex, string &triangleInfo,
										  const vector<pair<glm::vec3, glm::vec3>> &lineequations, vec3 center)
{
	vector<glm::vec3> fundamentalTriangle(3);
	for (int j = 0; j < lineequations.size(); j++)
	{
		for (int k = 0; k < j; k++)
		{
			if (!glm::isnan(approximateLineLineIntersect(lineequations[j], lineequations[k], fundamentalTriangle[0])))
			{
				for (int l = 0; l < k; l++)
				{
					if (!glm::isnan(approximateLineLineIntersect(lineequations[j], lineequations[l], fundamentalTriangle[1])) &&
						!glm::isnan(approximateLineLineIntersect(lineequations[k], lineequations[l], fundamentalTriangle[2])))
					{
						if (pointInsideTriangle(center, fundamentalTriangle) > M_EPSILON)
						{
							cutterIndex[0] = j;
							cutterIndex[1] = k;
							cutterIndex[2] = l;
							triangleInfo += string("fundamental triangle\n");
							triangleInfo += to_string(fundamentalTriangle[0]) + '\n';
							triangleInfo += to_string(fundamentalTriangle[1]) + '\n';
							triangleInfo += to_string(fundamentalTriangle[2]) + '\n';
							cout << cutterIndex[0] << " " << cutterIndex[1] << " " << cutterIndex[2] << endl;
							return fundamentalTriangle;
						}
					}
				}
			}
		}
	}
	assert(false);
	return fundamentalTriangle;
}

// return the triangularized vertices for a unit sphere
vector<glm::vec3> getSphereVertices(vector<glm::vec3> & normals, vector<int> & planeOffsets, vector<string> & triangleInfo, 
		vector<glm::vec3> & lineEqs, int nPlanes, float minRadius, float variance){
	// assume there's no randomness at first since that complicates stuff
	float salt=minRadius*variance;
	float radius = glm::linearRand(minRadius, minRadius+salt);

	// planes have the same normal and point. so 1 array is enough	
	vector<glm::vec3> planes;
	// 6 fundamental planes, to make sure we get a closed shape, approximately a cube
	// actually random ones will probably just do fine
	planes.push_back(vec3(0, 0, radius));
	planes.push_back(vec3(0, 0, -radius));
	planes.push_back(vec3(0, radius, 0));
	planes.push_back(vec3(0, -radius, 0));
	planes.push_back(vec3(radius, 0, 0));
	planes.push_back(vec3(-radius, 0, 0));

	// rotate those 4 planes randomly so they don't cause nan errors bc they are parallel to an axis
	vec3 m_rotation_axis = glm::sphericalRand(1.0);
	for(int i = 0; i < planes.size(); i++){
		planes[i] = glm::rotate(planes[i], 0.5f , m_rotation_axis);
		assert( glm::l2Norm(planes[i]) <= radius+M_EPSILON && glm::l2Norm(planes[i]) >= radius-M_EPSILON);
		planes[i] = glm::rotate(planes[i], glm::linearRand(0.01f, 0.1f) , m_rotation_axis);
	}

	while( planes.size()<nPlanes){
		planes.push_back(glm::sphericalRand(radius));
	}

	vector<glm::vec3> triangles;
	// after creating the planes, find their intersections... easier said than done
	for(int i=0; i<planes.size(); i++){
		cout << "Creating Plane: " << i << endl;
		triangleInfo.push_back(string());
		vector<pair<glm::vec3, glm::vec3>> lineequations;
		vector<vector<glm::vec3>> tempTriangles;
		vec3 n0 = planes[i] / glm::l2Norm(planes[i]);
		for (int j=0; j<planes.size(); j++){
			// same plane
			if(i == j){
				continue;
			}

			pair<vec3,vec3> newLine = getPlanePlaneIntersect(planes[i], planes[j]);
			lineequations.push_back(newLine);
			vec3 epsilon = glm::sphericalRand(0.2);
			for (int lineTriangle=0; lineTriangle<3; lineTriangle++){
				// root
				lineEqs.push_back(newLine.second-newLine.first*2.0f);
				// spine
				vec3 spine = newLine.second+newLine.first*2.0f;
				// skins
				vec3 noise = glm::rotate(epsilon, 2*glm::pi<float>()*lineTriangle/3.0f, normalize(newLine.first));
				lineEqs.push_back(spine);
				lineEqs.push_back(spine+noise);
			}
		}
		
		int cutterIndex[3] = {-1,-1,-1};
		vector<glm::vec3> fundamentalTriangle = findFundamentalTriangle(cutterIndex, triangleInfo[i], lineequations, planes[i]);
		tempTriangles.push_back(fundamentalTriangle);
		

		for (int j=0; j<lineequations.size(); j++){
			if(j==cutterIndex[0]||j==cutterIndex[1]||j==cutterIndex[2]){
				continue;
			}
			// cut all existing triangles based on the new line
			triangleInfo[i] += string("cut happening with:\n");
			triangleInfo[i] += to_string(lineequations[j].first) + '\n';
			triangleInfo[i] += to_string(lineequations[j].second) + '\n';
			// basically restarts the function until a correct polyhedra is returned
			if(!cutTriangles(lineequations[j], tempTriangles, planes[i])){
				triangleInfo.clear();
				lineEqs.clear();
				return getSphereVertices(normals, planeOffsets, triangleInfo, lineEqs, nPlanes, minRadius, variance);
			}
			cout << "triangle count after cut:" << tempTriangles.size() << endl;
		}
		for (int j = 0; j<tempTriangles.size(); j++){
			if( glm::dot (n0, glm::cross(tempTriangles[j][1]-tempTriangles[j][0], tempTriangles[j][2]-tempTriangles[j][0]) ) < 0.0){
				swap(tempTriangles[j][1], tempTriangles[j][2]);
			}
			for (int k=0; k<3; k++){
				triangles.push_back(tempTriangles[j][k]);
				normals.push_back(n0);
			}
		}
		planeOffsets.push_back(triangles.size());
	}

	for(int i=0; i<triangles.size(); i++){
		for(int j=0; j<i; j++){
			if(all(epsilonEqual(triangles[i],triangles[j], M_EPSILON))){
				triangles[i] = triangles[j];
				break;
			}
		}
	}

	cout<< "# triangles:\t" << triangles.size() << endl;
	return triangles;
}

void getPQPRotationMatrixFromQuat(PQP_REAL rmat[3][3], const quat & glmquat, mat3 & glmMat){
	glmMat = mat3_cast(glmquat);
	// mat3 matColOrder = transpose( glmMat);
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			rmat[i][j] = glmMat[j][i];
		}
	}
}

inline void moveOnTime(quat & rangle, vec3 & rloc, const quat & angle, const vec3 & loc, const vec4 & angVel, const vec3 & vel, float deltaTime){
	rloc = loc + vel*deltaTime;
	rangle = glm::rotate(angle, angVel[3]*deltaTime, vec3(angVel));
}


void collisionGivenTimeStep(
	PQP_CollideResult &cres, quat *angles, glm::vec3 *locations, PQP_Model *models, int i, int j,
	float deltaTime, float m_TimeStep, vec3 *velocity, vec4 *angularVelocity, int collisionCheckType)
{
	glm::vec3 curLocs[2];
	quat curAngles[2];

	moveOnTime(curAngles[0], curLocs[0], angles[i], locations[i], angularVelocity[i], velocity[i], m_TimeStep-deltaTime);
	moveOnTime(curAngles[1], curLocs[1], angles[j], locations[j], angularVelocity[j], velocity[j], m_TimeStep-deltaTime);

	PQP_REAL r1mat [3][3];
	PQP_REAL r2mat [3][3];
	mat3 r1glmMat;
	mat3 r2glmMat;
	
	getPQPRotationMatrixFromQuat(r1mat, curAngles[0], r1glmMat);
	getPQPRotationMatrixFromQuat(r2mat, curAngles[1], r2glmMat);

	PQP_Collide(&cres,r1mat, (PQP_REAL*)value_ptr(curLocs[0]),&models[i],
		r2mat,(PQP_REAL*)value_ptr(curLocs[1]),&models[j], collisionCheckType);
}


vec3 getCollisionVertex(vector<vec3> & vertices, PQP_CollideResult & cres, const mat3 & matV, const mat3 & matT, vec3 locV, vec3 locT, 
						vec3 vtriangle[3], bool triangleInFirstShape){
	//cout <<"finding collision vertex" <<endl;
	float maxPenetration = -1.0f;
	vec3 collisionVertex(0.0f);

	vec3 triangleCentroid;
	for(int i=0; i<3; i++){
		triangleCentroid+=vtriangle[i];
	}
	triangleCentroid/=3.0f;
	triangleCentroid = matT*triangleCentroid + locT;

	vec3 triangleNormal = matT * normalize(cross(vtriangle[1] - vtriangle[0], vtriangle[2] - vtriangle[0]));
	
	/*cout << to_string(triangleNormal) << endl;
	cout << to_string(triangleCentroid) << endl;*/


	for(int i=0; i< cres.NumPairs(); i++){
		int tid = triangleInFirstShape? cres.Id2(i) : cres.Id1(i);
		for (int j=0; j<3; j++){
			vec3 candidateVertex = vertices[tid*3+j];
			vec3 candidateWorld = matV*candidateVertex+locV;
			float penetration = dot(triangleCentroid-candidateWorld, triangleNormal);
			/*cout << "penetration and candidate:\n";
			cout << penetration << endl;
			cout << to_string(candidateWorld) << endl;*/
			if (penetration > maxPenetration){
				maxPenetration = penetration;
				collisionVertex = candidateVertex;
			}
		}
	}
	// return 0 vector if error
	return collisionVertex;
}

// it doesn't actually count triangle ids but anyways
// instead of countign triangles new version counts the # faces, how about that!
int cntDiffTriangleIds(PQP_CollideResult & cres, bool modelId, int diffIds[3], vector<glm::vec3> & vertices){
	// init array just in case
	diffIds[0] = -1;
	diffIds[1] = -1;
	diffIds[2] = -1;

	vec3 normals[3];

	int diffTriangleCnt = 0;
	// cout << "#pairs\t" << cres.NumPairs() << endl;
	vec3 commonVertex;

	// well we have at least one triangle!
	if(cres.NumPairs()){
		diffTriangleCnt=1;
	}

	for(int i=0; i<cres.NumPairs(); i++){
		
		int tid;

		if (modelId){
			tid = cres.Id2(i);
		}
		else{
			tid = cres.Id1(i);
		}

		vec3 inormal = normalize(cross(vertices[tid*3+1]-vertices[tid*3], vertices[tid*3+2]-vertices[tid*3]));

		if(diffTriangleCnt==1){
			diffIds[0] = tid;
		}
		// we have found a 3rd face involved
		else if(diffTriangleCnt==2){
			if(l1Norm(inormal-normals[0])>M_EPSILON && l1Norm(inormal-normals[1])>M_EPSILON){
				diffIds[2]=i;
				diffTriangleCnt=3;
				break;
			}
		}

		for (int j = 0; j < i && diffTriangleCnt==1; j++)
		{
			int jid;

			if (modelId)
			{
				jid = cres.Id2(j);
			}
			else
			{
				jid = cres.Id1(j);
			}

			vec3 jnormal = normalize(cross(vertices[jid * 3 + 1] - vertices[jid * 3], vertices[jid * 3 + 2] - vertices[jid * 3]));
			/*cout << "printing the normals for triangles:\t" << tid << "\t" << jid << endl;
			cout << to_string(inormal) << endl;
			cout << to_string(jnormal) << endl;*/
			if (l1Norm(inormal - jnormal) > M_EPSILON)
			{
				// we add our second face
				// check if they share a line
				for (int ii = 0; ii < 3; ii++)
				{
					for (int jj = 0; jj < 3; jj++)
					{
						// they share a line
						float mdot = dot(normalize(vertices[tid * 3 + (ii + 1) % 3] - vertices[tid * 3 + ii]), normalize(vertices[jid * 3 + (jj + 1) % 3] - vertices[jid * 3 + jj]));
						if (abs(mdot) > 1 - M_EPSILON)
						{
							diffIds[0] = tid;
							diffIds[1] = jid;
							diffTriangleCnt = 2;
							normals[0] = inormal;
							normals[1] = jnormal;
						}
					}
				}
			}
		}
	}
	// cout << diffTriangleCnt << endl;
	return diffTriangleCnt;
}

int getCollisionType(PQP_CollideResult & cres, int triIdsRes[3], vector<glm::vec3> & vertices0, vector<glm::vec3> & vertices1){
	int triIds[3];
	int n0 = cntDiffTriangleIds(cres, false, triIds, vertices0);
	if(n0>2){		
		for(int i=0; i<3; i++){
			triIdsRes[i] = triIds[i];
		}
	}
	// early termination case
	if(n0==0){
		return 0;
	}

	int n1 = cntDiffTriangleIds(cres, true, triIds, vertices1);
	if(n1>2){		
		for(int i=0; i<3; i++){
			triIdsRes[i] = triIds[i];
		}
	}

	cout << "checking collision type" << endl;
	cout << n0 << "\t" << n1 << endl;
	// face-vertex
	if( n0==1 && n1>2){
		return 1;
	}
	// vertex face
	else if(n1==1 && n0>2){
		return 2;
	}
	// edge-edge
	else if(n0 == 2 && n1 == 2){
		return 3;
	}
	// too many collisions to identify
	else if(n1+n0>4){
		return 4;
	}
	// not enough collisions
	else{
		return 0;
	}
}


// given two triangles that are known to share an overlapping line, return the line between them
pair<vec3, vec3> findLineBetweenTriangles(vector<vec3> & v, int tid[3]){
	for(int i=0; i<3; i++){
		vec3 m0 = normalize( v[tid[0]*3+i] - v[tid[0]*3+(i+1)%3]);
		for(int j=0; j<3; j++){
			vec3 m1 = normalize(v[tid[1]*3+j] - v[tid[1]*3+(j+1)%3]);
			float d = dot(m0,m1);
			if(abs(d) > 1.0-M_EPSILON){
				return pair(m0, v[tid[0]*3+(i+1)%3]);
			}
		}
	}
	assert(false);
	return pair(vec3(), vec3());
}

// return the closest point and the normal between two planes given two planes made an edge-edge collision
void getEdgeEdgeCollisionVertex(vec3 & ra, vec3 & rb, vec3 & collisionNormal, int tids0[3], int tids1[3],
		vector<vec3> & v0, vector<vec3> & v1, vec3 toiLoc0, vec3 toiLoc1, mat3 toiMat1, mat3 toiMat2){
	pair<vec3, vec3> line0 = findLineBetweenTriangles(v0, tids0);
	pair<vec3, vec3> line1 = findLineBetweenTriangles(v1, tids1); 

	pair<vec3, vec3> line0World(toiMat1*line0.first, toiMat1*line0.second + toiLoc0);
	pair<vec3, vec3> line1World(toiMat2*line1.first, toiMat2*line1.second + toiLoc1);
	
	// applying the answer found here
	// https://math.stackexchange.com/a/2812513 
	// and here with the sign fixed version 
	// https://math.stackexchange.com/a/4347294
	float r1 = length2(line0World.first);
	float r2 = length2(line1World.first);

	float d4321 = dot(line0World.first, line1World.first);
	float d3121 = dot(line1World.second-line0World.second, line0World.first);
	float d4331 = dot(line1World.second-line0World.second, line1World.first);
    
	float den = d4321*d4321 - r1*r2;

	float s = (d4321*d4331 - r2*d3121)/den;
    float t = (r1*d4331 - d4321*d3121)/den;

	collisionNormal = normalize(cross(line0World.first, line1World.first));
	// make sure normal points towards the second object
	if(dot(collisionNormal, toiLoc1 - toiLoc0) < 0.0f){
		collisionNormal *= -1.0f;
	}

	cout << "edge2edge collision point:\t" << to_string((s*line0World.first+line0World.second + t*line1World.first+line1World.second)/2.0f) << endl;

	// find the actual collision points in world coordinates, and then convert to model frame (without rotation) 
	ra = s*line0World.first+line0World.second - toiLoc0;
	rb = t*line1World.first+line1World.second - toiLoc1;
}

// vector<glm::vec3> vertices[maxObjCnt]? does this copy all the elements in the vertex vectors ???
int doPhysics(PQP_Model* models, vector<glm::vec3> vertices[maxObjCnt], glm::vec3* locations,
		 quat * angles, vec3 * velocity, vec4 * angularVelocity, int objCount, float deltaTime, float fastCollision, vector<float> collision_info[maxObjCnt],
		 mat3* inertiaTensor, mat3* inverseInertia)
{

	float forceCoeff = 0.98f;
	// euler integration
	// last object is not touched because it's the wall
	for (int i = 0; i < objCount-1; i++){
		locations[i] += velocity[i]*deltaTime;
		angles[i] = glm::rotate(angles[i], angularVelocity[i][3]*deltaTime, vec3(angularVelocity[i]));
		for(int j=0; j<collision_info[i].size(); j++){
			collision_info[i][j] = 0.0f;
		}
	}
	
	vec3 deltaVelocity[maxObjCnt];
	vec3 deltaAngularVelocity[maxObjCnt];

	// do collisions in here
	PQP_REAL r1mat [3][3];
	PQP_REAL r2mat [3][3];
	mat3 r1glmMat;
	mat3 r2glmMat;

	PQP_CollideResult cres;

	// sphere vs sphere collisions
	for(int i=0; i<polyhedraCnt; i++){
		getPQPRotationMatrixFromQuat(r1mat, angles[i], r1glmMat);

		for(int j=0; j<i; j++){
			if(fastCollision && fastCollision < l2Norm(locations[i]-locations[j])){
				continue;
			}

			getPQPRotationMatrixFromQuat(r2mat, angles[j], r2glmMat);


			PQP_Collide(&cres,r1mat, (PQP_REAL*)value_ptr(locations[i]),&models[i],
						r2mat,(PQP_REAL*)value_ptr(locations[j]),&models[j], PQP_ALL_CONTACTS);
			
			int collisionType = 0;
			int triangleIdsForVertex[3];

			collisionType = getCollisionType(cres, triangleIdsForVertex, vertices[i], vertices[j]);

			// if there's a collision, find the actual collision time
			if(collisionType){
				float minCollisionTime = 0.0f;
				float maxCollisionTime = deltaTime;
				float m_TimeStep;

				// sanity check, if there is a collision at the beginning of this timestep, we failed to do impulse in the previous step
				// to make it more robust, just fix it and move the objects back in time.
				for(int backInTime=0; backInTime<1; backInTime++){
					minCollisionTime = -backInTime*deltaTime;
					collisionGivenTimeStep(cres, angles, locations, models, i, j,
												deltaTime, minCollisionTime, velocity, angularVelocity, PQP_ALL_CONTACTS);
					
					collisionType = getCollisionType(cres, triangleIdsForVertex, vertices[i], vertices[j]);
					if(!collisionType){
						if(backInTime>0){
							cout << "**contact at the beginning of update**\t"<< i<<'\t'<< j<< endl;
							cout << "back in time iteration:\t" << backInTime << endl;
						}
						break;
					}
				}
				
				if(collisionType){
					cout << "**contact at the beginning of update error**\t"<< i<<'\t'<< j<< endl;
					for(int k=0; k<cres.NumPairs(); k++){
						for(int l=0; l<3; l++){
							collision_info[i][cres.Id1(k)*3+l] = 1.0;
							collision_info[j][cres.Id2(k)*3+l] = 1.0;
						}
					}
					return i*polyhedraCnt+j;
				}

				// do the basic search for 5 steps
				for(int bi=0; bi<5; bi++){
					m_TimeStep = (minCollisionTime+maxCollisionTime)/2.0f;
					
					collisionGivenTimeStep(cres, angles, locations, models, i, j,
												deltaTime, m_TimeStep, velocity, angularVelocity, PQP_ALL_CONTACTS);
					collisionType = getCollisionType(cres, triangleIdsForVertex, vertices[i], vertices[j]);

					if(collisionType){
						maxCollisionTime = m_TimeStep;
					}
					else{
						minCollisionTime = m_TimeStep;
					}
				}


				
				// check for loop condition and save its result
				while(((collisionType = getCollisionType(cres, triangleIdsForVertex, vertices[i], vertices[j]))==0 || collisionType==4)){
					m_TimeStep = (minCollisionTime+maxCollisionTime)/2.0f;
					if(m_TimeStep-minCollisionTime<=0.0f || maxCollisionTime-m_TimeStep<=0.0f){
						
						for(int k=0; k<cres.NumPairs(); k++){
							for(int l=0; l<3; l++){
								collision_info[i][cres.Id1(k)*3+l] = 1.0;
								collision_info[j][cres.Id2(k)*3+l] = 1.0;
							}
						}

						cout << "ERROR iteration ended \t" << i <<"\t" << j << endl;
						return i*polyhedraCnt+j;
					}
					
					collisionGivenTimeStep(cres, angles, locations, models, i, j,
												deltaTime, m_TimeStep, velocity, angularVelocity, PQP_ALL_CONTACTS);

					if(collisionType){
						maxCollisionTime = m_TimeStep;
					}
					else{
						minCollisionTime = m_TimeStep;
					}
					cout << "iteration timestep\t" << m_TimeStep << endl;
					cout << "num collision\t" << cres.NumPairs() << endl;
					cout << "collision type\t" << collisionType << endl;
					cout << "wings \t" << maxCollisionTime << '\t' << minCollisionTime << endl;
				}
				// collision point from the center of two objects
				vec3 ra(0.0f);
				vec3 rb(0.0f);
				// always points from first obj to 2nd one
				// actually in the slides it's the reverse but think about this later
				vec3 collisionNormal;

				
				for(int k=0; k<cres.NumPairs(); k++){
					for(int l=0; l<3; l++){
						collision_info[i][cres.Id1(k)*3+l] = 1.0;
						collision_info[j][cres.Id2(k)*3+l] = 1.0;
					}
				}

				moveOnTime(angles[i], locations[i], angles[i], locations[i], angularVelocity[i], velocity[i], m_TimeStep-deltaTime);
				moveOnTime(angles[j], locations[j], angles[j], locations[j], angularVelocity[j], velocity[j], m_TimeStep-deltaTime);

				mat3 toiMat1(angles[i]); 
				mat3 toiMat2(angles[j]);

				// vertex face
				if( collisionType == 2){
					// first polyhedron has the vertex
					vec3 rawCollisionVertex = getCollisionVertex(vertices[i], cres, toiMat1, toiMat2, locations[i], locations[j], &vertices[j][cres.Id2(0)*3], false);
					if(l1Norm(rawCollisionVertex)==0.0f){
						return i*polyhedraCnt+j;
					}
					ra = toiMat1 * rawCollisionVertex;
					rb = ra + locations[i] - locations[j];
					collisionNormal =  - toiMat2 * normalize(cross(vertices[j][cres.Id2(0)*3+1]-vertices[j][cres.Id2(0)*3+0], vertices[j][cres.Id2(0)*3+2]-vertices[j][cres.Id2(0)*3+0]));
				}
				// second has the vertex
				else if(collisionType == 1){
					vec3 rawCollisionVertex = getCollisionVertex(vertices[j], cres, toiMat2, toiMat1, locations[j], locations[i], &vertices[i][cres.Id1(0)*3], true);
					if(l1Norm(rawCollisionVertex)==0.0f){
						return i*polyhedraCnt+j;
					}
					rb = toiMat2 * rawCollisionVertex;
					ra = rb - locations[i] + locations[j];
					collisionNormal = toiMat1 * normalize(cross(vertices[i][cres.Id1(0)*3+1]-vertices[i][cres.Id1(0)*3+0], vertices[i][cres.Id1(0)*3+2]-vertices[i][cres.Id1(0)*3+0]));
				}
				else if(collisionType == 3){
					int triangleIdsForP0[3];
					cntDiffTriangleIds(cres, false, triangleIdsForP0, vertices[i]);
					int triangleIdsForP1[3];
					cntDiffTriangleIds(cres, true, triangleIdsForP1, vertices[j]);
					getEdgeEdgeCollisionVertex(ra, rb, collisionNormal, triangleIdsForP0, triangleIdsForP1, 
													vertices[i], vertices[j], locations[i], locations[j], toiMat1, toiMat2);
				}
				else{
					cout << "ERROR couldn't identify collision type" << endl;
					return i*polyhedraCnt+j;
				}


				// previous steps return the normal from A to B but it should be from B to A
				collisionNormal *= -1.0f;

				vec3 pointVelocityA = velocity[i] + cross (vec3(angularVelocity[i]) * angularVelocity[i][3], ra);
				vec3 pointVelocityB = velocity[j] + cross (vec3(angularVelocity[j]) * angularVelocity[j][3], rb);
				float relativeContactVelocity = dot(collisionNormal, pointVelocityA-pointVelocityB);

				mat3 iInverseInertia = toiMat1 * inverseInertia[i] * transpose(toiMat1);
				mat3 jInverseInertia = toiMat2 * inverseInertia[j] * transpose(toiMat2);

				float jImpact = -relativeContactVelocity*(1.0f+forceCoeff);
				jImpact /= 2.0f + dot(collisionNormal, cross((iInverseInertia*cross(ra, collisionNormal)), ra))
						+ dot(collisionNormal, cross((jInverseInertia*cross(rb, collisionNormal)), rb));
				
				deltaVelocity[i] += collisionNormal*jImpact;
				deltaVelocity[j] -= collisionNormal*jImpact;
				deltaAngularVelocity[i] += cross(ra, collisionNormal*jImpact);
				deltaAngularVelocity[j] += cross(collisionNormal*jImpact, rb);

				cout << "SUCCESSSSS!\t" << i <<"\t" << j << endl;
				if (verbose){
					cout << "collision between:\t"<< i << "\t" << j << endl;
				}
				cout << "****start shitty debug****\n";
				cout << "type of collision:\t" << collisionType << '\n';
				cout << "locations\n" << to_string(locations[i]) << '\n' << to_string(locations[j]) << '\n';
				cout << "\nrelative locations\n" << to_string(ra) << '\n' << to_string(rb) << '\n';
				cout << "normal\t" << to_string(collisionNormal);
				cout << "\n\n point velocities:\n" << to_string(pointVelocityA) << '\n' << to_string(pointVelocityB) << endl;
				cout << "contact velocity\t" << relativeContactVelocity;
				cout << "\njimpact\t" << jImpact;
				cout << "\nvel change\t" << to_string(collisionNormal*jImpact) << endl;
				cout << "****end shitty debug****\n";
				
			}

			
		}
		// handling the wall
		getPQPRotationMatrixFromQuat(r2mat, angles[polyhedraCnt], r2glmMat);
		PQP_CollideResult cres;
			PQP_Collide(&cres,r1mat, (PQP_REAL*)value_ptr(locations[i]),&models[i],
						r2mat,(PQP_REAL*)value_ptr(locations[polyhedraCnt]),&models[polyhedraCnt], PQP_FIRST_CONTACT);
		
		
		if(cres.NumPairs()){
			// handle the change of speed
			vec3 triangleNormal = normalize(cross(vertices[polyhedraCnt][cres.Id2(0)*3+1]-vertices[polyhedraCnt][cres.Id2(0)*3],
												vertices[polyhedraCnt][cres.Id2(0)*3+2]-vertices[polyhedraCnt][cres.Id2(0)*3]));
			if(dot(velocity[i],triangleNormal)<0.0f ){
				deltaVelocity[i] -= (1.0f+forceCoeff)*dot(velocity[i],triangleNormal)*triangleNormal;
				deltaAngularVelocity[i] -= (1.0f+forceCoeff)*vec3(angularVelocity[i])*angularVelocity[i][3];
				if(any(isnan(deltaVelocity[i])) || any(isnan(deltaAngularVelocity[i]))){
					cout << to_string(deltaVelocity[i]) << endl;
					cout << to_string(deltaAngularVelocity[i]) << endl;
					cout << to_string(triangleNormal) << endl;
					cout << to_string(velocity[i]) << endl;
					cout << to_string(angularVelocity[i]) << endl;
					return i;
				}

				moveOnTime(angles[i], locations[i], angles[i], locations[i], angularVelocity[i], velocity[i], -deltaTime);


				cout << "wall collision:\t" << i << endl;
				}
			else{
				cout << "continuing wall collision:\t" << i << endl;
			}

			
		}
		

	}
	// cout <<"new velocities\n";
	for (int i = 0; i < polyhedraCnt; i++){
		velocity[i] += deltaVelocity[i];
		getPQPRotationMatrixFromQuat(r1mat, angles[i], r1glmMat);

		vec3 newAngularForce = r1glmMat * inertiaTensor[i] * transpose(r1glmMat) * vec3(angularVelocity[i])*angularVelocity[i][3] + deltaAngularVelocity[i];
		newAngularForce = r1glmMat * inverseInertia[i] * transpose(r1glmMat) * newAngularForce;
		float newForceMagnitude = l2Norm(newAngularForce);
		angularVelocity[i] = vec4(newAngularForce/newForceMagnitude, newForceMagnitude);

		if(any(isnan(deltaAngularVelocity[i])) || any(isnan(deltaVelocity[i]))){
				cout << to_string(deltaVelocity[i]) << endl;
				cout << to_string(deltaAngularVelocity[i]) << endl;
				cout << to_string(velocity[i]) << endl;
				cout << to_string(angularVelocity[i]) << endl;
				cout << to_string(locations[i]) << endl;
				cout << to_string(angles[i]) << endl;
			return 1;
		}
		/*cout << to_string(velocity[i]) << endl;
		cout << to_string(angularVelocity[i]) << endl;
		assert(l2Norm(velocity[i])<5000.0);*/
	}
	return 0;
}


// return the triangularized vertices for a unit cube
vector<glm::vec3> getRoomVertices(vector<glm::vec3> & normals, float length){
	
	// discarding the top section

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


void performUnitTests(){
	// line intersection tests
	pair<vec3,vec3> l;
	l.first=vec3(0.5, 0.5, 0);
	l.second=vec3(0.0, 0.0, 0.0);
	pair<vec3,vec3> l1;
	l1.first=vec3(1.0, -1.0, 0);
	l1.second=vec3(0.0, 1.0, 0.0);
	vec3 p;
	assert(!glm::isnan(approximateLineLineIntersect(l, l1, p)));
	assert( (glm::l2Norm( p-vec3(0.5, 0.5, 0.0) ) ) < 0.01 );
	
	// plane plane intersection tests
	vec3 plane0(0,0,1);
	vec3 plane1(0,1,0);
	l = getPlanePlaneIntersect(plane0, plane1);
	assert(glm::l2Norm( l.first - vec3(-1,0,0)) == 0.0);
	assert(glm::l2Norm( l.second - vec3(0,1,1)) == 0.0);

	plane0 = vec3(0,0,1);
	plane1 = vec3(0,sqrt(3.0f)/2.0,0.5);
	l = getPlanePlaneIntersect(plane0, plane1);
	assert(glm::dot ( l.first, vec3(-1,0,0))/l2Norm(l.first)  == 1.0);
	assert(glm::l2Norm( l.second - vec3(0,1.0f/sqrt(3),1)) < 0.0001);

	// triangle intersections
	vector<vec3> t0;
	t0.push_back(vec3(0.0));
	t0.push_back(vec3(0.9,1.0,0.0));
	t0.push_back(vec3(1.0,0.1,0.0));

	vec3 t0Centroid(0.0);
	for(int i=0; i < 3; i++)
		t0Centroid+=t0[i];
	t0Centroid/=3.0;
	vector<vec3> t1;

	for(int i=0; i < 3; i++)
		t1.push_back( (t0Centroid+t0[i])/2.0f );
	
	assert(triangleTriangleOverlap(t0,t1));
	t1[1] = t0[1];
	t1[2] = t0[2];
	assert(triangleTriangleOverlap(t0,t1));
	
	t1[0] =  t0Centroid + (t0Centroid-t0[0]);
	assert(!triangleTriangleOverlap(t0,t1));

	t1[1] *= 0.8f;
	assert(triangleTriangleOverlap(t0,t1));
}



int main( int argc, char * argv[] )
{

	float roomRadius = 25.0f;
	int avgFaceCnt;
	float apprRadius;
	float variance = 0.75;
	double runTimeMax = 0;
	// first arg is # of polygons
	if(argc > 1){
		polyhedraCnt = atoi(argv[1]);
		avgFaceCnt = atoi(argv[2]);
		apprRadius = atof(argv[3]);
		variance = atof(argv[4]);
		runTimeMax = atof(argv[5]);
	}
	else{
		polyhedraCnt = 10;
		avgFaceCnt = 15;
		apprRadius = 1.0f;
	}
	objCount = polyhedraCnt+1;

	assert(polyhedraCnt < pow(roomRadius/(2.0f*apprRadius), 3.0f));

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
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS); 

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// perform "unit tests"
	performUnitTests();

	float speedMultiplier = 1.0;



	vector<string> triangleInfo[maxObjCnt];
	vector<glm::vec3> normals[maxObjCnt];
	vector<vec3> lineEqs[maxObjCnt];
	vector<glm::vec3> vertices[maxObjCnt];
	vector <int> planeOffsets[maxObjCnt];
	vector<float> triangleEdges[maxObjCnt];
	vector<float> bcollision[maxObjCnt];

	vertices[polyhedraCnt] = getRoomVertices(normals[polyhedraCnt], roomRadius);

	// last buffer is saved for the future prediction
	vec3 locationsBackup[backupBufferSize+1][maxObjCnt];
	quat anglesBackup[backupBufferSize+1][maxObjCnt];

	vec3 locations[maxObjCnt];
	quat angles[maxObjCnt];

	vec3 velocity[maxObjCnt];
	vec4 angularVelocity[maxObjCnt];
	mat3 inertiaTensors[maxObjCnt];
	mat3 inverseInertia[maxObjCnt];
	cout << to_string(vec3(roomRadius/2.0f)) << endl;

	vec3 specularColor[maxObjCnt];
	vec3 diffuseColor[maxObjCnt];
	PQP_Model pqpmodels[maxObjCnt];

	uint16_t xGroups = round(pow(polyhedraCnt, 1.0f/3.0f));
	uint16_t yGroups = xGroups;
	uint16_t zGroups = polyhedraCnt/(xGroups*yGroups);
	if (xGroups*yGroups*zGroups < polyhedraCnt){
		zGroups++;
	}

	float xInterval = 2.0/xGroups;
	float zInterval = 2.0/zGroups;
	float yInterval = xInterval;

	float maxR = 0;

	for(int i=0; i < polyhedraCnt; i++){
		
		do{
			planeOffsets[i].clear();
			if(triangleInfo[i].size() > 0){
				cout << "creating polyhedra again after unsuccessful attempt" << endl;
			}
			triangleInfo[i].clear();
			lineEqs[i].clear();
			normals[i].clear();

			planeOffsets[i].push_back(0);
			vertices[i] = getSphereVertices(normals[i], planeOffsets[i], triangleInfo[i], lineEqs[i], avgFaceCnt, apprRadius, variance);
			
			pqpmodels[i] = PQP_Model();
			pqpmodels[i].BeginModel();
			for(int j=0; j<vertices[i].size(); j+=3){
				PQP_REAL points[9];
				for(int k=0; k<9; k++){
					points[k] = vertices[i][j+k/3][k%3];
					if(k%3==0){
						float vertNorm = l2Norm(vertices[i][j+k/3]);
						if(vertNorm>maxR){
							maxR = vertNorm;
						}
					}
				}
				pqpmodels[i].AddTri(&points[0], &points[3], &points[6], j/3);
			}
		}while (pqpmodels[i].EndModel());

		uint16_t xId = i/(polyhedraCnt/xGroups);
		uint16_t yId = i%(polyhedraCnt/xGroups)/(polyhedraCnt/yGroups/xGroups);
		uint16_t zId = i%(polyhedraCnt/yGroups/xGroups);
		float xLoc = xId*xInterval-1.0f+xInterval/2.0f;
		float yLoc = yId*yInterval-1.0f+yInterval/2.0f;
		float zLoc = zId*zInterval-1.0f+zInterval/2.0f;

		locations[i] = vec3(roomRadius/2.0f)*vec3(xLoc, yLoc, zLoc);
		cout << to_string(locations[i]) << endl;
		velocity[i] = normalize(ballRand(1.0f) - (length2(locations[i]) ? normalize(locations[i]):vec3(0.0)));
		angularVelocity[i] = vec4(sphericalRand(1.0f), linearRand(0.0f, 1.0f));

		specularColor[i] = sphericalRand(1.0f);
		diffuseColor[i] = (vec3(1.0)-specularColor[i])/2.0f;

		for (int j = 0; j < planeOffsets[i].size(); j++){
			while(triangleEdges[i].size() < planeOffsets[i][j]){
				float triVal = glm::linearRand(0.0f,1.0f);
				triangleEdges[i].push_back(triVal);
				triangleEdges[i].push_back(triVal);
				triangleEdges[i].push_back(triVal);
			}
		}

		inertiaTensors[i] = getInertiaTensorForPolyhedra(vertices[i]);
		inverseInertia[i] = inverse(inertiaTensors[i]);
		/*cout << "printing inertias:\n";
		cout << to_string(inertiaTensors[i]) << endl;
		cout << to_string(inverseInertia[i]) << endl;*/
	}
	specularColor[polyhedraCnt] = vec3(0.7);
	diffuseColor[polyhedraCnt] = vec3(0.1);
	pqpmodels[polyhedraCnt].BeginModel();
	cout << vertices[polyhedraCnt].size()<< endl;
	for (int j = 0; j < vertices[polyhedraCnt].size(); j += 3)
	{
		PQP_REAL points[9];
		for (int k = 0; k < 9; k++)
		{
			points[k] = vertices[polyhedraCnt][j + k / 3][k % 3];
		}
		pqpmodels[polyhedraCnt].AddTri(&points[0], &points[3], &points[6], j / 3);
	}

	for(int i=0; i< objCount; i++){
		for(int j=0; j<vertices[i].size();j++){
			bcollision[i].push_back(0.0);
		}
	}

	pqpmodels[polyhedraCnt].EndModel();
	cout << "model" << endl;

	int renderedPlaneCnt = 0;
	


	GLuint vertexbuffers[maxObjCnt];
	glGenBuffers(objCount, vertexbuffers);
	GLuint normalbuffers[maxObjCnt];
	glGenBuffers(objCount, normalbuffers);

	GLuint triangleEdgeBuffer[maxObjCnt];
	glGenBuffers(objCount, triangleEdgeBuffer);

	GLuint collision_info[maxObjCnt];
	glGenBuffers(objCount, collision_info);

	GLuint mLineBuffer[maxObjCnt];
	glGenBuffers(objCount, mLineBuffer);

	for(int i=0; i < objCount; i++){
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers[i]);
		glBufferData(GL_ARRAY_BUFFER, vertices[i].size() * sizeof(glm::vec3), &vertices[i][0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, normalbuffers[i]);
		glBufferData(GL_ARRAY_BUFFER, normals[i].size() * sizeof(glm::vec3), &normals[i][0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, triangleEdgeBuffer[i]);
		glBufferData(GL_ARRAY_BUFFER, triangleEdges[i].size() * sizeof(float), &triangleEdges[i][0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, collision_info[i]);
		glBufferData(GL_ARRAY_BUFFER, bcollision[i].size() * sizeof(float), &bcollision[i][0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, mLineBuffer[i]);
		glBufferData(GL_ARRAY_BUFFER, lineEqs[i].size() * sizeof(glm::vec3), &lineEqs[i][0], GL_STATIC_DRAW);
	}
	

	GLuint m_shaderProgram = LoadShaders( "GeneralPurposeVertexShader.glsl", "GeneralPurposeFragmentShader.glsl" );
	GLuint edgeDebugProgram = LoadShaders( "EdgeVertex.glsl", "EdgeFragment.glsl" );

	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(m_shaderProgram, "MVP");
	GLuint MatrixMVID = glGetUniformLocation(m_shaderProgram, "MV");
	GLuint diffuseColorID = glGetUniformLocation(m_shaderProgram, "diffuseColor");
	GLuint specularColorID = glGetUniformLocation(m_shaderProgram, "specularColor");

	GLuint MatrixIDEdge = glGetUniformLocation(edgeDebugProgram, "MVP");
	double preTime = glfwGetTime();
	// Always check that our framebuffer is ok
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		return false;

	bool pressable[4] = {true,true,true,true};
	double totalTime = 0;
	unsigned long long int frameCntr=0;
	unsigned long long int frozenFrame=0;
	bool firstIteration = true;
	double timeSpentOnPhysics = 0;
 

	setControlsWidthHeight(windowWidth, windowHeight);

	int freezePhysics=0;
	bool renderingCurTime[2]={true,true};
	do{
		// Render to the screen
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
        // Render on the whole framebuffer, complete from the lower left corner to the upper right
		glViewport(0,0,windowWidth,windowHeight);

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Compute the MVP matrix from keyboard and mouse input
		computeMatricesFromInputs();
		if(totalTime < 2.0){
			setCamera(vec3(0.0f,0.0f,-20.0f), vec3(0.0f,0.0f,1.0f));	
		}
		glm::mat4 ProjectionMatrix = getProjectionMatrix();
		glm::mat4 ViewMatrix = getViewMatrix();
		double currentTime = glfwGetTime();
		double deltaTime = currentTime-preTime;
		preTime = currentTime;
		if(!firstIteration){
			totalTime+=deltaTime;
			frameCntr++;
			// end the execution
			if(runTimeMax && runTimeMax <= totalTime){
				cout <<  "\navg fps is:\n" << frameCntr/totalTime << endl;
				cout <<  "\navg fps for the physics is:\n" << frameCntr/timeSpentOnPhysics << endl;
				break;
			}
		}
		else{
			firstIteration = false;
		}

		if(!freezePhysics){
			memcpy(&locationsBackup[frameCntr%backupBufferSize][0], &locations[0], sizeof(locations));
			memcpy(&anglesBackup[frameCntr%backupBufferSize][0], &angles[0], sizeof(angles));
		}

		float fastCollision = variance ? maxR*2 : variance;
		double measure = glfwGetTime();
		if(!freezePhysics){
			freezePhysics = doPhysics(pqpmodels, vertices, locations, angles, velocity, angularVelocity, 
							objCount, deltaTime* speedMultiplier, fastCollision, bcollision, inertiaTensors, inverseInertia);
			if(freezePhysics){
				frozenFrame=frameCntr+1;
				vec3 obj0Loc = locations[freezePhysics/polyhedraCnt];
				vec3 obj1Loc = locations[freezePhysics%polyhedraCnt];
				cout << to_string( obj0Loc ) << "\t" << to_string(obj1Loc) << endl;
				vec3 collisionCenter = (obj1Loc+obj0Loc)/2.0f;
				vec3 camLocationDir = cross(obj1Loc-obj0Loc, vec3(0.0,1.0,0.0));
				setCamera(camLocationDir+collisionCenter, normalize(-camLocationDir));

				for(int i=0; i<polyhedraCnt; i++){
					moveOnTime(anglesBackup[backupBufferSize][i], locationsBackup[backupBufferSize][i], angles[i], locations[i], angularVelocity[i], velocity[i], deltaTime*backupBufferSize);
				}
			}
		}

		if(freezePhysics){
			if (glfwGetKey( window, GLFW_KEY_T ) == GLFW_PRESS && pressable[2]){
				renderingCurTime[0] = !renderingCurTime[0];
				pressable[2] = false;
				cout << "is not rendering previous timestep:\t" << renderingCurTime[0] << endl;
			}
			else if (glfwGetKey( window, GLFW_KEY_T ) == GLFW_RELEASE){
				pressable[2] = true;
			}
			
			if (glfwGetKey( window, GLFW_KEY_Y ) == GLFW_PRESS && pressable[3]){
				renderingCurTime[1] = !renderingCurTime[1];
				pressable[3] = false;
				cout << "is not rendering future timestep:\t" << renderingCurTime[1] << endl;
			}
			else if (glfwGetKey( window, GLFW_KEY_Y ) == GLFW_RELEASE){
				pressable[3] = true;
			}
		}
		else{
			if (glfwGetKey( window, GLFW_KEY_T ) == GLFW_PRESS && pressable[2]){
				pressable[2] = false;
				cout << "printing info:\n";
				for(int i=0; i<polyhedraCnt;i++){
					cout << to_string(locations[i]) << '\n'; 
					cout << to_string(velocity[i]) << '\n';
					cout << to_string(angles[i]) << '\n';
					cout << to_string(angularVelocity[i]) << '\n';
				}
			}
			else if (glfwGetKey( window, GLFW_KEY_T ) == GLFW_RELEASE){
				pressable[2] = true;
			}
		}

		timeSpentOnPhysics += glfwGetTime() - measure;

		// Use our shader
		glUseProgram(m_shaderProgram);

		for (int i = 0; i < objCount; i++){

			// Build the model matrix
			glm::mat4 ModelMatrix;
			if(freezePhysics && !(renderingCurTime[0] && renderingCurTime[1])){
				if(!renderingCurTime[0]){
					ModelMatrix = glm::translate(glm::mat4(), locationsBackup[frozenFrame%backupBufferSize][i]) *  mat4_cast(anglesBackup[frozenFrame%backupBufferSize][i]);
				}
				else{
					ModelMatrix = glm::translate(glm::mat4(), locationsBackup[backupBufferSize][i]) *  mat4_cast(anglesBackup[backupBufferSize][i]);
				}
			}
			else{
				ModelMatrix = glm::translate(glm::mat4(), locations[i]) *  mat4_cast(angles[i]);
			}
			
			glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
			glm::mat4 MV = ViewMatrix * ModelMatrix;


			// Send our transformation to the currently bound shader, 
			// in the "MVP" uniform
			glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
			glUniformMatrix4fv(MatrixMVID, 1, GL_FALSE, &MV[0][0]);
			glUniform3f(specularColorID, specularColor[i][0], specularColor[i][1], specularColor[i][2]);
			glUniform3f(diffuseColorID, diffuseColor[i][0], diffuseColor[i][1], diffuseColor[i][2]);
			glUniformMatrix4fv(MatrixMVID, 1, GL_FALSE, &MV[0][0]);
			
			// 1rst attribute buffer : vertices
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers[i]);
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
			glBindBuffer(GL_ARRAY_BUFFER, normalbuffers[i]);
			glVertexAttribPointer(
				1,                                // attribute
				3,                                // size
				GL_FLOAT,                         // type
				GL_FALSE,                         // normalized?
				0,                                // stride
				(void*)0                          // array buffer offset
			);

			//3rd attribute debug blues
			glEnableVertexAttribArray(2);
			glBindBuffer(GL_ARRAY_BUFFER, collision_info[i]);
			glBufferSubData(GL_ARRAY_BUFFER, 0, bcollision[i].size() * sizeof(float), &bcollision[i][0]);
			glVertexAttribPointer(
				2,                                // attribute
				1,                                // size
				GL_FLOAT,                         // type
				GL_FALSE,                         // normalized?
				0,                                // stride
				(void*)0                          // array buffer offset
			);

			// Draw the triangle !
			glDrawArrays(GL_TRIANGLES, 0, vertices[i].size()); // draw all triangles
			//sphereVertices.size()); // draw all triangles
			//planeOffsets[renderedPlaneCnt]);

			glDisableVertexAttribArray(0);
			glDisableVertexAttribArray(1);
			glDisableVertexAttribArray(2);
		}


		/*
		if (glfwGetKey( window, GLFW_KEY_N ) == GLFW_PRESS && pressable[0]){
			renderedPlaneCnt = glm::clamp(--renderedPlaneCnt, 0, (int)planeOffsets.size()-1);
			pressable[0] = false;
			if(renderedPlaneCnt>0)
			cout <<"rendering:" << renderedPlaneCnt <<"\n"  << triangleInfo[renderedPlaneCnt-1] << endl;
		}
		else if (glfwGetKey( window, GLFW_KEY_N ) == GLFW_RELEASE){
			pressable[0] = true;
		}
		if (glfwGetKey( window, GLFW_KEY_M ) == GLFW_PRESS && pressable[1]){
			renderedPlaneCnt = glm::clamp(++renderedPlaneCnt, 0, (int)planeOffsets.size()-1);
			pressable[1] = false;
			if(renderedPlaneCnt>0)
			cout <<"rendering:" << renderedPlaneCnt-1 <<"\n"  << triangleInfo[renderedPlaneCnt-1] << endl;
		}
		else if (glfwGetKey( window, GLFW_KEY_M ) == GLFW_RELEASE){
			pressable[1] = true;
		} */
		
		/*
		glUseProgram(edgeProgram);
		
		glUniformMatrix4fv(MatrixIDEdge, 1, GL_FALSE, &MVP[0][0]);
		// draw separating lines for debugging
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, mLineBuffer);
		glVertexAttribPointer(
			0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);

		glDrawArrays(GL_TRIANGLES, 0, 90 //lineEqs.size()
		); // draw all triangles

		glDisableVertexAttribArray(0);*/


		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();


	} // Check if the ESC key was pressed or the window was closed
	while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
		   glfwWindowShouldClose(window) == 0 );

	// Cleanup VBO and shader
	glDeleteBuffers(1, vertexbuffers);
	glDeleteProgram(m_shaderProgram);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return 0;
}

