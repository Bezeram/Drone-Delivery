#pragma once

#include "components/simple_scene.h"
#include "core/gpu/mesh.h"
#include "lab_m1/DroneGames/Camera.h"
#include "lab_m1/DroneGames/Terrain.h"
#include "lab_m1/DroneGames/Utils.h"

#include <iostream>
#include <vector>
#include <cstdlib>

namespace m1
{
	class DroneGames : public gfxc::SimpleScene
	{
	public:
		DroneGames();

		void Init() override;

		void RenderMesh(Mesh* mesh, Shader* shader, const glm::mat4& modelMatrix) override;
		void RenderArrowMesh(Mesh* mesh, Shader* shader, const glm::mat4& modelMatrix);
		void RenderTerrainMesh(Mesh* mesh, Shader* shader, const glm::mat4& modelMatrix);
		bool m_IsDroneCollidingWithTree(const glm::vec3& dronePosition);
		Mesh* CreateSquare(const std::string& name, glm::vec3 leftBottomCorner, float length, glm::vec3 color, bool fill = true);
		Mesh& CreateRectangle3D(const std::string& name, glm::vec3 leftBottomCorner, float length, float width, float height, glm::vec3 color, bool fill = true);
		void Render(float deltaTimeSeconds);
		void SpawnNewCargoPosition();

		void FrameStart() override;
		void Update(float deltaTimeSeconds) override;
		void FrameEnd() override;

		void OnInputUpdate(float deltaTime, int mods) override;
		void OnKeyPress(int key, int mods) override;
		void OnMouseMove(int mouseX, int mouseY, int deltaX, int deltaY) override;
		void OnMouseBtnPress(int mouseX, int mouseY, int button, int mods) override;
		void OnMouseBtnRelease(int mouseX, int mouseY, int button, int mods) override;
		void OnMouseScroll(int mouseX, int mouseY, int offsetX, int offsetY) override;
		void OnWindowResize(int width, int height) override;
	private:
		float m_CalculateArrowRotation();
		bool m_IsDroneCollidingWithCargo();
		bool m_IsDroneCollidingWithBunker();
		void m_HandleDroneInputs(float deltaTime, int mods);
		void m_HandleTerrainEditorInputs(float deltaTime, int mods);
		void m_GenerateFlatMesh(size_t resolution, float length, const char* meshName);
		void m_GenerateTerrainMesh();
		void m_GenerateBunkers();
	private:
		float m_AccumulatorUpdate = 0;
		size_t TPS = 64;

		dg::Terrain m_Terrain;
		float m_TerrainResolution = dg::Defaults::TerrainResolution;
		float m_TerrainResolutionChangeSpeed = 30.f;
		float m_TerrainNoiseOffsetSpeed = 3;
		float m_TerrainNoiseScalarSpeed = 0.1;
		float m_TerrainHeightScalarSpeed = 1;
		float m_TerrainLengthChangeSpeed = 30.f;
		
		dg::Camera m_Camera;
		glm::mat4 m_ProjectionMatrix = glm::mat4(1);

		float left = -5.0f, right = 5.0f;
		float bottom = -5.0f, top = 5.0f;
		float nearPlane = 0.01f, farPlane = 200.0f;

		float multiplier_proj = 1.0f;
		float nearPlaneTranslate = 0.f;
		float farPlaneTranslate = 0.f;

		const char* m_TerrainMeshName = "terrain";
		const char* m_TerrainShaderName = "terrain_shader";
		const char* m_TreeTrunkShaderName = "tree_trunk_shader";
		const char* m_TreeLeavesShaderName = "tree_leaves_shader";
		const char* m_GuidingArrowShaderName = "guiding_arrow_shader";

		const char* m_DroneCubeMesh = "drone_cube";
		const char* m_DroneRotorMesh = "drone_rotor";
		const char* m_GuidingArrowMesh = "guiding_arrow";
		const char* m_BunkerMesh = "bunker";

		float m_GravityNormal = 0.2f;
		float m_GravityDive = 1.6f;
		float m_GravityCargo = 2.f;

		// Drone vars
		glm::vec3 m_DronePreviousPosition = glm::vec3(0.f);
		float m_RotorSpeed = 30.f;
		float m_RotorsAngle = 0.f;
		float m_DroneMaxSpeed = 0.5f;
		float m_DroneUpwardsSpeed = 0.f;
		float m_DroneAcceleration = 0.f;
		// For drone collisions
		float m_DroneForwardSpeed = 0.f;
		const float m_DronePushbackSpeed = 0.15f;
		size_t m_DroneCollisions = 0;
		// Horizontal tilt and Vertical tilt respectively
		// Sinus of the angle results in the amount of tilt
		glm::vec2 m_DroneTiltAngle = { 0, 0 };
		glm::vec2 m_DroneTiltScalar = { 0.03f, 0.1f };
		float m_DroneTiltSpeed = RADIANS(40);
		float m_DroneTiltDecay = RADIANS(30);
		float m_DroneMaxTiltAngle = RADIANS(20);
		float m_DroneThrustPower = 1.f;
		float m_DronePitchSpeed = 1.f;
		float m_DroneMaxPitchY = 0.9;
		float m_DroneTransportingHeightAccumulator = 0.f;
		float m_DroneTransportTimer = 0.f;
		float m_DroneTransportTotalTime = 0.f;

		// Transport
		glm::vec3 m_CargoFallPosition = { 0, 0, 0 };
		bool m_IsCargoDropped = false;
		bool m_IsCargoCollected = false;
		glm::vec3 m_CargoDronePosition = { 0, 0, 0 };
		glm::vec3 m_CargoCollectPosition = { 0, 0, 0 };
		float m_TargetRadius = 0.5f;
		glm::vec2 m_TargetPosition = glm::vec2(dg::Defaults::TerrainLength / 2);
		float m_CargoRadius = 0.3f;

		float m_BunkersDensity = 0.356f;
		glm::vec3 m_BunkerSize = { 5.f, 2.f, 3.5f };
		std::vector<glm::vec2> m_BunkersPositions;

		bool RENDER_WIREFRAME = false;

		bool m_IsCursorLocked = false;
	};
}