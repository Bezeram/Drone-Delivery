#pragma once

#include "lab_m1/DroneGames/Camera.h"

#include <iostream>

using namespace dg;

Camera::Camera()
	: m_DistanceToTarget(1.f)
	, m_Position(0, 2, 5)
	, m_Forward(0, 0, -1)
	, m_Up(0, 1, 0)
	, m_Right(1, 0, 0)
{}

Camera::Camera(const glm::vec3& position, const glm::vec3& center, const glm::vec3& up)
{
	Set(position, center, up);
}

// Update camera
void Camera::Set(const glm::vec3& position, const glm::vec3& center, const glm::vec3& up)
{
	this->m_Position = position;
	m_Forward = glm::normalize(center - position);
	m_Right	= glm::cross(m_Forward, up);
	this->m_Up = glm::cross(m_Right, m_Forward);
}

void Camera::SetTargetDistance(float distance)
{
	m_DistanceToTarget = distance;
}

float Camera::GetTargetDistance() const
{
	return m_DistanceToTarget;
}

void Camera::MoveUpward(float distance)
{
	glm::vec3 dir = glm::normalize(glm::vec3(0, m_Up.y, 0));
	m_Position += dir * distance;
}

void Camera::MoveForward(float distance)
{
	glm::vec3 dir = glm::normalize(glm::vec3(m_Forward.x, 0, m_Forward.z));
	m_Position += dir * distance;
	// movement will keep the camera at the same height always
	// Example: If you rotate up/down your head and walk forward you will still keep the same relative distance (height) to the ground!
	// Translate the camera using the DIR vector computed from forward
}

void Camera::TranslateForward(float distance)
{
	// Translate the camera using the "forward" vector
	m_Position += glm::normalize(m_Forward) * distance;
}

void Camera::TranslateUpward(float distance)
{
	// Translate the camera using the up vector
	m_Position += glm::normalize(m_Up) * distance;
}

void Camera::TranslateRight(float distance)
{
	// Translate the camera using the "right" vector
	// Usually translation using camera "right' is not very useful because if the camera is rotated around the "forward" vector 
	// translation over the right direction will have an undesired effect; the camera will get closer or farther from the ground
	// Using the projected right vector (onto the ground plane) makes more sense because we will keep the same distance from the ground plane]
	m_Position += glm::normalize(m_Right) * distance;
}

void Camera::RotateFirstPerson_OX(float angle)
{
	// Compute the new "forward" and "up" vectors
	// Attention! Don't forget to normalize the vectors
	// Use glm::rotate()
	glm::vec4 aux = glm::rotate(glm::mat4(1.f), angle, m_Right) * glm::vec4(m_Forward, 0);
	m_Forward = glm::normalize(glm::vec3(aux));
	m_Up = glm::cross(m_Right, m_Forward);
}

void Camera::RotateFirstPerson_OY(float angle)
{
	// Compute the new "forward", "up" and "right" vectors
	// Don't forget to normalize the vectors
	// Use glm::rotate()
	glm::vec4 aux = glm::rotate(glm::mat4(1.f), angle, glm::vec3(0, 1, 0)) * glm::vec4(m_Forward, 0);
	m_Forward = glm::normalize(glm::vec3(aux));

	aux = glm::rotate(glm::mat4(1.f), angle, glm::vec3(0, 1, 0)) * glm::vec4(m_Right, 0);
	m_Right = glm::normalize(glm::vec3(aux));

	m_Up = glm::cross(m_Right, m_Forward);
}

void Camera::RotateFirstPerson_OZ(float angle)
{
	// Compute the new Right and Up, Forward stays the same
	// Don't forget to normalize the vectors
	glm::vec4 aux = glm::rotate(glm::mat4(1.f), angle, glm::vec3(0, 1, 0)) * glm::vec4(m_Right, 1);
	m_Right = glm::normalize(glm::vec3(aux));

	aux = glm::rotate(glm::mat4(1.f), angle, m_Forward) * glm::vec4(m_Up, 0);
	m_Forward = glm::normalize(glm::vec3(aux));

	m_Up = glm::cross(m_Right, m_Forward);
}

void Camera::RotateThirdPerson_OX(float angle)
{
	// Rotate the camera in Third Person mode - OX axis
	// Use distanceToTarget as translation distance
	TranslateForward(m_DistanceToTarget);
	RotateFirstPerson_OX(angle);
	TranslateForward(-m_DistanceToTarget);
}

void Camera::RotateThirdPerson_OY(float angle)
{
	// Rotate the camera in Third Person mode - OY axis
	TranslateForward(m_DistanceToTarget);
	RotateFirstPerson_OY(angle);
	TranslateForward(-m_DistanceToTarget);
}

void Camera::RotateThirdPerson_OZ(float angle)
{
	// Rotate the camera in Third Person mode - OZ axis
	TranslateForward(m_DistanceToTarget);
	RotateFirstPerson_OZ(angle);
	TranslateForward(-m_DistanceToTarget);
}

glm::mat4 Camera::GetViewMatrix() const
{
	// Returns the View Matrix
	return glm::lookAt(m_Position, m_Position + m_Forward, m_Up);
}

glm::vec3 Camera::GetTargetPosition() const
{
	return m_Position + m_Forward * m_DistanceToTarget;
}

glm::vec3 dg::Camera::GetPosition() const
{
	return m_Position;
}

float Camera::GetYawRotation() const
{
	glm::vec3 cameraRight = glm::normalize(m_Right);

	// Measure the cosine of the angle
	// The Y component of the cross product gives the sign
	glm::vec3 worldRight = glm::vec3(1, 0, 0);
	float dot = glm::dot(cameraRight, worldRight);  
	float crossY = glm::cross(worldRight, cameraRight).y;

	return atan2(crossY, dot);
}

glm::vec3 Camera::GetRightVector() const
{
	return m_Right;
}

glm::vec3 Camera::GetForwardDirectionVector() const
{
	return glm::vec3(m_Forward.x, 0, m_Forward.z);
}

glm::vec3 Camera::GetForwardVector() const
{
	return m_Forward;
}

