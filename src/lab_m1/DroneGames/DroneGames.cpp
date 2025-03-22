#include "lab_m1/DroneGames/DroneGames.h"

using namespace m1;
namespace Defaults = dg::Defaults;

DroneGames::DroneGames()
    : m_Camera()
    , m_Terrain(Defaults::TerrainResolution, Defaults::TerrainLength)
{
}

void DroneGames::Init()
{
    m_Camera.Set(glm::vec3(0, 2, 3.5f), glm::vec3(0, 1, 0), glm::vec3(0, 1, 0));

    {
        Mesh* mesh = new Mesh("sphere");
        mesh->LoadMesh(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::MODELS, "primitives"), "sphere.obj");
        meshes[mesh->GetMeshID()] = mesh;
    }

    // Tree model
    {
        Mesh* mesh = new Mesh("tree_trunk");
        mesh->LoadMesh(PATH_JOIN(window->props.selfDir, SOURCE_PATH::M1, "DroneGames", "models"), "cylinder_thing.obj");
        meshes[mesh->GetMeshID()] = mesh;        
        
        mesh = new Mesh("tree_leaves");
        mesh->LoadMesh(PATH_JOIN(window->props.selfDir, SOURCE_PATH::M1, "DroneGames", "models"), "cone_high.obj");
        meshes[mesh->GetMeshID()] = mesh;
    }

    // Direction arrow
    {
        Mesh* mesh = new Mesh(m_GuidingArrowMesh);
        mesh->LoadMesh(PATH_JOIN(window->props.selfDir, SOURCE_PATH::M1, "DroneGames", "models"), "direction_arrow.glb");
        meshes[mesh->GetMeshID()] = mesh;
    }

    // Terrain
    {
        Shader* shader = new Shader(m_TerrainShaderName);
        shader->AddShader(PATH_JOIN(window->props.selfDir, SOURCE_PATH::M1, "DroneGames", "shaders", "TerrainVertexShader.glsl"), GL_VERTEX_SHADER);
        shader->AddShader(PATH_JOIN(window->props.selfDir, SOURCE_PATH::M1, "DroneGames", "shaders", "TerrainFragmentShader.glsl"), GL_FRAGMENT_SHADER);
        shader->CreateAndLink();
        shaders[shader->GetName()] = shader;
    }

    // Tree trunk
    {
        Shader* shader = new Shader(m_TreeTrunkShaderName);
        shader->AddShader(PATH_JOIN(window->props.selfDir, SOURCE_PATH::M1, "DroneGames", "shaders", "TreeTrunkVertexShader.glsl"), GL_VERTEX_SHADER);
        shader->AddShader(PATH_JOIN(window->props.selfDir, SOURCE_PATH::M1, "DroneGames", "shaders", "TreeTrunkFragmentShader.glsl"), GL_FRAGMENT_SHADER);
        shader->CreateAndLink();
        shaders[shader->GetName()] = shader;
    }

    // Tree leaves
    {
        Shader* shader = new Shader(m_TreeLeavesShaderName);
        shader->AddShader(PATH_JOIN(window->props.selfDir, SOURCE_PATH::M1, "DroneGames", "shaders", "TreeLeavesVertexShader.glsl"), GL_VERTEX_SHADER);
        shader->AddShader(PATH_JOIN(window->props.selfDir, SOURCE_PATH::M1, "DroneGames", "shaders", "TreeLeavesFragmentShader.glsl"), GL_FRAGMENT_SHADER);
        shader->CreateAndLink();
        shaders[shader->GetName()] = shader;
    }

    // Arrow shader
    {
        Shader* shader = new Shader(m_GuidingArrowShaderName);
        shader->AddShader(PATH_JOIN(window->props.selfDir, SOURCE_PATH::M1, "DroneGames", "shaders", "ArrowVertexShader.glsl"), GL_VERTEX_SHADER);
        shader->AddShader(PATH_JOIN(window->props.selfDir, SOURCE_PATH::M1, "DroneGames", "shaders", "ArrowFragmentShader.glsl"), GL_FRAGMENT_SHADER);
        shader->CreateAndLink();
        shaders[shader->GetName()] = shader;
    }

    // Drone parts
	CreateRectangle3D(m_DroneCubeMesh, glm::vec3(-0.5, -0.5, -0.5), 1, 1, 1, glm::vec3(0.3, 0.3, 0.3));
    CreateRectangle3D(m_BunkerMesh, glm::vec3(0, 0, 0), 1, 1, 1, glm::vec3(0.2, 0.2, 0.2));
	CreateRectangle3D(m_DroneRotorMesh, glm::vec3(-2.5, -0.5, -0.5), 5, 1, 1, glm::vec3(0.1, 0.1, 0.1));

    m_Terrain.GenerateTerrain();
	m_Terrain.GenerateTrees();
    m_GenerateTerrainMesh();

    // Spawn cargo
    SpawnNewCargoPosition();

    // Generate bunkers
    m_GenerateBunkers();

    // parameters, remove hardcodings of these parameters
    m_ProjectionMatrix = glm::perspective(RADIANS(60), window->props.aspectRatio, 0.01f, 200.0f);
}

float DroneGames::m_CalculateArrowRotation()
{
    glm::vec3 arrowDir;
    float partialLength = m_Terrain.GetLength() / (m_Terrain.GetResolution() - 1);
    if (m_IsCargoCollected)
    {
		float terrainOffset = m_Terrain.GetLength() / 2.f;
        glm::vec3 worldTargetPosition = glm::vec3(m_TargetPosition.x, 0, m_TargetPosition.y) - glm::vec3(terrainOffset, 0, terrainOffset);
        arrowDir = worldTargetPosition - glm::vec3(m_Camera.GetPosition().x, 0, m_Camera.GetPosition().z);
    }
    else
        arrowDir = glm::vec3(m_CargoCollectPosition.x, 0, m_CargoCollectPosition.z) - glm::vec3(m_Camera.GetPosition().x, 0, m_Camera.GetPosition().z);
    arrowDir = glm::normalize(arrowDir);

    glm::vec3 cameraForward = glm::normalize(glm::vec3(m_Camera.GetForwardDirectionVector()));
    float dot = glm::dot(cameraForward, arrowDir);
    float crossY = glm::cross(cameraForward, arrowDir).y;
    return atan2(crossY, dot);
}

bool DroneGames::m_IsDroneCollidingWithCargo()
{
    glm::vec3 dronePosition = m_Camera.GetTargetPosition();
    glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1), m_Camera.GetYawRotation(), glm::vec3(0, 1, 0));

    std::vector<glm::vec3> hitboxPoints = {
        dronePosition + glm::vec3(rotationMatrix * glm::vec4(0.14f, 0, 0.14f, 1)),
        dronePosition + glm::vec3(rotationMatrix * glm::vec4(-0.14f, 0, 0.14f, 1)),
        dronePosition + glm::vec3(rotationMatrix * glm::vec4(0.14f, 0, -0.14f, 1)),
        dronePosition + glm::vec3(rotationMatrix * glm::vec4(-0.14f, 0, -0.14f, 1))
    };

    for (const auto& point : hitboxPoints)
    {
	    if (glm::distance(m_CargoCollectPosition, point) < m_CargoRadius)
	    {
		    return true;
	    }
    }

    return false;
}

bool DroneGames::m_IsDroneCollidingWithBunker()
{
    // Compute the positions of the 4 hitbox points (center of each cube)
    glm::vec3 dronePosition = m_Camera.GetTargetPosition();
    glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1), m_Camera.GetYawRotation(), glm::vec3(0, 1, 0));

    std::vector<glm::vec3> hitboxPoints = {
        dronePosition + glm::vec3(rotationMatrix * glm::vec4(0.14f, 0, 0.14f, 1)),  
        dronePosition + glm::vec3(rotationMatrix * glm::vec4(-0.14f, 0, 0.14f, 1)), 
        dronePosition + glm::vec3(rotationMatrix * glm::vec4(0.14f, 0, -0.14f, 1)), 
        dronePosition + glm::vec3(rotationMatrix * glm::vec4(-0.14f, 0, -0.14f, 1))
    };

    for (const glm::vec2& bunkerPositionXOZ : m_BunkersPositions)
    {
		float partialLength = m_Terrain.GetLength() / (m_Terrain.GetResolution() - 1);
        glm::vec3 terrainOffset = glm::vec3(-m_Terrain.GetLength() / 2.f, 0, -m_Terrain.GetLength() / 2.f);
        glm::vec3 bunkerPosition = glm::vec3(-bunkerPositionXOZ.x, 0, -bunkerPositionXOZ.y) * partialLength - terrainOffset;
        glm::vec3 bunkerMin = bunkerPosition;
        glm::vec3 bunkerMax = bunkerPosition + m_BunkerSize;

        // Check if any of the hitbox points are inside the bunker
        for (const glm::vec3& point : hitboxPoints)
        {
            if (point.x >= bunkerMin.x && point.x <= bunkerMax.x &&
                point.y >= bunkerMin.y && point.y <= bunkerMax.y &&
                point.z >= bunkerMin.z && point.z <= bunkerMax.z)
            {
                // Collision detected
                return true;
            }
        }
    }

    // No collision detected
    return false;
}

bool DroneGames::m_IsDroneCollidingWithTree(const glm::vec3& dronePosition)
{
    const auto& treesPositions = m_Terrain.GetTreesPositions();
    float resolutionTerrain = m_Terrain.GetResolution();
    float step = m_Terrain.GetLength() / (resolutionTerrain - 1);

    // Loop through all trees to check collisions
    for (const glm::vec2& treePos : treesPositions)
    {
        // Calculate tree position
        glm::vec3 centerOffset = glm::vec3(-m_Terrain.GetLength() / 2.f, 0, -m_Terrain.GetLength() / 2.f);
        glm::vec3 treeBasePosition = -glm::vec3(treePos.x, 0, treePos.y) * step - centerOffset;

        // Tree trunk collision (cylinder)
        float trunkRadius = 0.07f; // As rendered
        float trunkHeight = 0.5f; // As rendered
        glm::vec3 treeTrunkTop = treeBasePosition + glm::vec3(0, trunkHeight, 0);

        glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1), m_Camera.GetYawRotation(), glm::vec3(0, 1, 0));
        std::vector<glm::vec3> hitboxPoints = {
            dronePosition + glm::vec3(rotationMatrix * glm::vec4(0.14f, 0, 0.14f, 1)),
            dronePosition + glm::vec3(rotationMatrix * glm::vec4(-0.14f, 0, 0.14f, 1)),
            dronePosition + glm::vec3(rotationMatrix * glm::vec4(0.14f, 0, -0.14f, 1)),
            dronePosition + glm::vec3(rotationMatrix * glm::vec4(-0.14f, 0, -0.14f, 1))
        };

        for (const glm::vec3& point : hitboxPoints)
        {
            // Check collision with the cylindrical trunk
            float distanceToTrunk = glm::distance(glm::vec2(point.x, point.z), glm::vec2(treeBasePosition.x, treeBasePosition.z));
            if (distanceToTrunk <= trunkRadius && point.y >= treeBasePosition.y && point.y <= treeTrunkTop.y)
            {
                return true; // Collision with the trunk
            }

            // Tree leaves collision (cone)
            float coneHeight = 3.0f; // As rendered
            float coneBaseRadius = 0.8f; // As rendered
            glm::vec3 coneApex = treeTrunkTop + glm::vec3(0, coneHeight, 0);

            // Calculate collision with the cone
            glm::vec3 coneToDrone = point - coneApex;
            float verticalDistance = glm::dot(coneToDrone, glm::vec3(0, -1, 0)); // Height along cone's axis

            if (verticalDistance >= 0 && verticalDistance <= coneHeight)
            {
                float coneRadiusAtHeight = coneBaseRadius * (1.0f - (verticalDistance / coneHeight));
                glm::vec2 coneBaseToDrone = glm::vec2(coneToDrone.x, coneToDrone.z);
                if (glm::length(coneBaseToDrone) <= coneRadiusAtHeight)
                {
                    return true; // Collision with the leaves
                }
            }
        }
    }

    return false; // No collision detected
}


void DroneGames::Update(float deltaTimeSeconds)
{
    m_AccumulatorUpdate += deltaTimeSeconds;

    if (m_IsCargoCollected)
    {
        m_DroneTransportTimer += deltaTimeSeconds;
        m_DroneTransportTotalTime += deltaTimeSeconds;

        // Every 1 second, update the height accumulator
        while (m_DroneTransportTimer > 1.f)
        {
            m_DroneTransportTimer -= 1.f;

            m_DroneTransportingHeightAccumulator += m_Camera.GetTargetPosition().y;
        }
    }

    float fixedDeltaTime = 1.0f / TPS;
    while (m_AccumulatorUpdate > fixedDeltaTime)
    {
        m_AccumulatorUpdate -= fixedDeltaTime;

        m_HandleDroneInputs(fixedDeltaTime, 0);

        // Check for collisions
        if (m_IsDroneCollidingWithBunker())
        {
            glm::vec3 droneDirection = m_Camera.GetTargetPosition() - m_DronePreviousPosition;
			glm::vec3 cameraOrientation = m_Camera.GetForwardVector();
            m_DroneForwardSpeed = m_DronePushbackSpeed * -glm::dot(glm::normalize(droneDirection), glm::normalize(cameraOrientation)); // Push the drone back
            m_DroneCollisions++;
        }

        // Tree collision detection
        if (m_IsDroneCollidingWithTree(m_Camera.GetTargetPosition()))
        {
            glm::vec3 droneDirection = m_Camera.GetTargetPosition() - m_DronePreviousPosition;
            glm::vec3 cameraOrientation = m_Camera.GetForwardVector();
            m_DroneForwardSpeed = (m_DronePushbackSpeed / 1.5) * -glm::dot(glm::normalize(droneDirection), glm::normalize(cameraOrientation));
            m_DroneCollisions++;
        }

        if (!m_IsCargoDropped)
        {
            if (!m_IsCargoCollected && m_IsDroneCollidingWithCargo())
            {
                m_IsCargoCollected = true;
            }
        }
        else
            m_CargoFallPosition.y -= deltaTimeSeconds * m_GravityCargo;

        // Check cargo collision with terrain
        float terrainOffset = m_Terrain.GetLength() / 2.f;
        float partialLength = m_Terrain.GetLength() / (m_Terrain.GetResolution() - 1);
        glm::vec2 cargoPositionXOZ = glm::vec2(m_CargoFallPosition.x, m_CargoFallPosition.z + terrainOffset) / partialLength;
        size_t index = cargoPositionXOZ.y * m_Terrain.GetResolution() + cargoPositionXOZ.x;
        if (m_IsCargoDropped && m_Terrain.GetHeightMap()[index] - 2 * m_CargoRadius > m_CargoFallPosition.y)
        {
            float terrainOffset = m_Terrain.GetLength() / 2.f;
            glm::vec3 cargoPositionXOZ = glm::vec3(m_CargoFallPosition.x, 0, m_CargoFallPosition.z);
            glm::vec3 targetPositionXOZ = glm::vec3(m_TargetPosition.x, 0, m_TargetPosition.y) - glm::vec3(terrainOffset, 0, terrainOffset);
            float distanceToTarget = glm::distance(cargoPositionXOZ, targetPositionXOZ);

            float score = 0;
            float averageDroneHeight = 0;
            if (distanceToTarget < 3 * m_TargetRadius)
            {
                averageDroneHeight = 5.f - m_DroneTransportingHeightAccumulator / m_DroneTransportTotalTime;
                score = float(m_DroneCollisions) * -5 + averageDroneHeight * 20 + distanceToTarget * 2;
            }

            if (score == 0)
            {
                std::cout << "Target missed! score 0" << std::endl;
            }
            else
            {
				std::cout << "Collisions deduction: " << float(m_DroneCollisions) * -5 << std::endl;
				std::cout << "Height score: " << averageDroneHeight * 20 << std::endl;
				std::cout << "Distance from target score: " << distanceToTarget * 2 << std::endl;
                std::cout << "Final score: " << score << std::endl << "//--------------//";
            }

            // Reset status variables
            m_DroneCollisions = 0;
            m_IsCargoCollected = false;
            m_IsCargoDropped = false;
            m_DroneTransportTimer = 0.f;
            m_DroneTransportingHeightAccumulator = 0.f;
            m_DroneTransportTotalTime = 0.f;
            SpawnNewCargoPosition();
        }

        // Drone movement updates
        {
            m_DronePreviousPosition = m_Camera.GetTargetPosition();
            m_DroneForwardSpeed -= glm::sign(m_DroneForwardSpeed) * fixedDeltaTime * 0.15;
            if (m_DroneForwardSpeed > -0e-4 && m_DroneForwardSpeed < 0e-4) // Below an error threshold erase the contribution
            {
				m_DroneForwardSpeed = 0;
            }

            float gravity = (window->KeyHold(GLFW_KEY_LEFT_SHIFT)) ? m_GravityDive : m_GravityNormal;

            m_DroneUpwardsSpeed += m_DroneAcceleration * fixedDeltaTime - gravity * fixedDeltaTime;
            m_DroneUpwardsSpeed = dg::ClampMax(m_DroneUpwardsSpeed, m_DroneMaxSpeed);
            float upwardsMovement = m_DroneUpwardsSpeed * fixedDeltaTime;
            m_Camera.MoveUpward(upwardsMovement);

            if (m_Camera.GetTargetPosition().y > 10.f)
                m_Camera.MoveUpward(-upwardsMovement);

            float tilt = sin(m_DroneTiltAngle.y) * m_DroneTiltScalar.y + m_DroneForwardSpeed;
            m_Camera.MoveForward(tilt);

            tilt = sin(m_DroneTiltAngle.x) * m_DroneTiltScalar.x;
            m_Camera.RotateThirdPerson_OY(tilt);
        }
    }

    Render(deltaTimeSeconds);
}

void DroneGames::Render(float deltaTimeSeconds)
{
	GLenum polygonMode = RENDER_WIREFRAME ? GL_LINE : GL_FILL;
    glPolygonMode(GL_FRONT_AND_BACK, polygonMode);
    glCullFace(GL_FRONT);

    // Render the drone.
    {
		glm::mat4 tiltTransform = glm::rotate(glm::mat4(1), m_DroneTiltAngle.x, -glm::normalize(m_Camera.GetForwardDirectionVector()));
        tiltTransform = glm::rotate(tiltTransform, m_DroneTiltAngle.y, -glm::normalize(m_Camera.GetRightVector()));

        // Long rectangles
        {
            glm::mat4 modelMatrix = glm::mat4(1);
            modelMatrix = glm::translate(modelMatrix, m_Camera.GetTargetPosition());
			modelMatrix *= tiltTransform;
            modelMatrix = glm::rotate(modelMatrix, m_Camera.GetYawRotation() + float(M_PI / 4), glm::vec3(0, 1, 0));
            modelMatrix = glm::scale(modelMatrix, glm::vec3(0.02f, 0.02f, 0.3f));
            RenderMesh(meshes[m_DroneCubeMesh], shaders["VertexColor"], modelMatrix);
        }
        {
            glm::mat4 modelMatrix = glm::mat4(1);
            modelMatrix = glm::translate(modelMatrix, m_Camera.GetTargetPosition());
            modelMatrix *= tiltTransform;
		    modelMatrix = glm::rotate(modelMatrix, m_Camera.GetYawRotation() - float(M_PI / 4), glm::vec3(0, 1, 0));
            modelMatrix = glm::scale(modelMatrix, glm::vec3(0.02f, 0.02f, 0.3f));
            RenderMesh(meshes[m_DroneCubeMesh], shaders["VertexColor"], modelMatrix);
        }
        // Cubes at the ends
		for (int dx = -1; dx <= 1; dx += 2)
		{
			for (int dz = -1; dz <= 1; dz += 2)
			{
                glm::vec3 rotationOffset = { dx * 0.14 * cos(M_PI / 4), 0.02, dz * 0.14 * sin(M_PI / 4) };

				glm::mat4 modelMatrix = glm::mat4(1);
				modelMatrix = glm::translate(modelMatrix, m_Camera.GetTargetPosition() + glm::vec3(0, 2 * 0.02, 0));
                // Rotate around the center of the drone
                modelMatrix *= tiltTransform;
                modelMatrix = glm::rotate(modelMatrix, m_Camera.GetYawRotation(), glm::vec3(0, 1, 0));
				modelMatrix = glm::translate(modelMatrix, -rotationOffset);
				modelMatrix = glm::scale(modelMatrix, glm::vec3(-0.02f, -0.02f, -0.02f));
				RenderMesh(meshes[m_DroneCubeMesh], shaders["VertexColor"], modelMatrix);
			}
		}
		// Rotors
		m_RotorsAngle += deltaTimeSeconds * m_RotorSpeed;
        for (int dx = -1; dx <= 1; dx += 2)
        {
            for (int dz = -1; dz <= 1; dz += 2)
            {
                glm::vec3 rotationOffset = { dx * 0.14 * cos(M_PI / 4), 0.02, dz * 0.14 * sin(M_PI / 4) };

                glm::mat4 modelMatrix = glm::mat4(1);
                modelMatrix = glm::translate(modelMatrix, m_Camera.GetTargetPosition() + glm::vec3(0, 2 * 0.02 + 0.01, 0));
                // Rotate around the center of the drone
                modelMatrix *= tiltTransform;
                modelMatrix = glm::rotate(modelMatrix, m_Camera.GetYawRotation(), glm::vec3(0, 1, 0));
                modelMatrix = glm::translate(modelMatrix, -rotationOffset);
                // Rotate around itself
                modelMatrix = glm::rotate(modelMatrix, m_RotorsAngle * dx * dz, glm::vec3(0, 1, 0));
                modelMatrix = glm::scale(modelMatrix, glm::vec3(0.02f, 0.01f, 0.02f));
                RenderMesh(meshes[m_DroneRotorMesh], shaders["VertexColor"], modelMatrix);
            }
        }
    }

    // Bunkers
    {
        glm::vec3 terrainOffset = glm::vec3(-m_Terrain.GetLength() / 2.f, 0, -m_Terrain.GetLength() / 2.f);
        
        float resolutionTerrain = m_Terrain.GetResolution();
        float step = m_Terrain.GetLength() / (resolutionTerrain - 1);
        for (const glm::vec2& pos : m_BunkersPositions)
        {
            glm::vec3 renderPos = -glm::vec3(pos.x, 0, pos.y) * step - terrainOffset;

            glm::mat4 modelMatrix = glm::mat4(1);
            modelMatrix = glm::translate(modelMatrix, renderPos);
			modelMatrix = glm::scale(modelMatrix, m_BunkerSize);
            RenderMesh(meshes[m_BunkerMesh], shaders["VertexColor"], modelMatrix);
        }
    }

	// Render the terrain
	{
        float terrainLength = m_Terrain.GetLength();

		glm::mat4 modelMatrix = glm::mat4(1);
		modelMatrix = glm::translate(modelMatrix, glm::vec3(-terrainLength / 2.f, 0, -terrainLength / 2.f));
		RenderTerrainMesh(meshes[m_TerrainMeshName], shaders[m_TerrainShaderName], modelMatrix);
	}

    // Render trees
    {
		glm::vec3 centerOffset = glm::vec3(-m_Terrain.GetLength() / 2.f, 0, -m_Terrain.GetLength() / 2.f);

		const auto& treesPositions = m_Terrain.GetTreesPositions();
        const auto& heightMap = m_Terrain.GetHeightMap();
        float resolutionTerrain = m_Terrain.GetResolution();
        float step = m_Terrain.GetLength() / (resolutionTerrain - 1);
        for (const glm::vec2& treePos : treesPositions)
        {
            glm::vec3 renderPos = -glm::vec3(treePos.x, 0, treePos.y) * step - centerOffset;

            // Skip optimization if tree is close enough on XOZ plane.
            // This is a quick dirty fix. Frustum culling is the real solution here.
            if (glm::distance(glm::vec3(renderPos.x, 0, renderPos.z), glm::vec3(m_Camera.GetPosition().x, 0, m_Camera.GetPosition().z)) > 10.f)
            {
                // Check if it's in front of the camera
                // If it isn't, skip rendering it
                glm::vec3 cameraPosXOZ = glm::vec3(m_Camera.GetPosition().x, 0, m_Camera.GetPosition().z);
                glm::vec3 treePosXOZ = glm::vec3(renderPos.x, 0, renderPos.z);
                glm::vec3 cameraTreeDir = glm::normalize(treePosXOZ - cameraPosXOZ);
                if (glm::dot(cameraTreeDir, glm::normalize(m_Camera.GetForwardVector())) < 0)
                    continue;
            }
            
            glm::mat4 modelMatrixTrunk = glm::mat4(1);
			modelMatrixTrunk = glm::translate(modelMatrixTrunk, renderPos);
			modelMatrixTrunk = glm::scale(modelMatrixTrunk, glm::vec3(0.1f, 0.5f, 0.1f));
			RenderMesh(meshes["tree_trunk"], shaders[m_TreeTrunkShaderName], modelMatrixTrunk);

			glm::mat4 modelMatrixLeaves = glm::mat4(1);
            modelMatrixLeaves = glm::translate(modelMatrixLeaves, renderPos);
			modelMatrixLeaves = glm::translate(modelMatrixLeaves, glm::vec3(0, 3, 0));
            modelMatrixLeaves = glm::scale(modelMatrixLeaves, glm::vec3(0.8f, 0.8f, 0.8f));
			RenderMesh(meshes["tree_leaves"], shaders[m_TreeLeavesShaderName], modelMatrixLeaves);
			modelMatrixLeaves= glm::translate(modelMatrixLeaves, glm::vec3(0, 1, 0));
            modelMatrixLeaves = glm::scale(modelMatrixLeaves, glm::vec3(0.8f, 0.8f, 0.8f));
            RenderMesh(meshes["tree_leaves"], shaders[m_TreeLeavesShaderName], modelMatrixLeaves);
			modelMatrixLeaves= glm::translate(modelMatrixLeaves, glm::vec3(0, 1, 0));
            modelMatrixLeaves = glm::scale(modelMatrixLeaves, glm::vec3(0.8f, 0.8f, 0.8f));
            RenderMesh(meshes["tree_leaves"], shaders[m_TreeLeavesShaderName], modelMatrixLeaves);
        }
    }

	// Render cargo
    {
        glm::vec3 renderPos;
        
        if (!m_IsCargoDropped)
            renderPos = (m_IsCargoCollected) ? m_Camera.GetTargetPosition() - glm::vec3(0, m_CargoRadius / 2 + 0.02f, 0) : m_CargoCollectPosition;
        else
            renderPos = m_CargoFallPosition;

        glm::mat4 modelMatrix = glm::mat4(1);
        modelMatrix = glm::translate(modelMatrix, renderPos);
        modelMatrix = glm::rotate(modelMatrix, (float)Engine::GetElapsedTime(), glm::vec3(0, 1, 0));
        modelMatrix = glm::scale(modelMatrix, glm::vec3(m_CargoRadius));
        RenderMesh(meshes["sphere"], shaders["VertexNormal"], modelMatrix);
    }

    // Render arrow
    {
        glm::vec3 renderPos = m_Camera.GetTargetPosition() + glm::normalize(m_Camera.GetForwardDirectionVector()) * -0.5f;
		float angle = m_CalculateArrowRotation();

        glm::mat4 modelMatrix = glm::mat4(1);
        modelMatrix = glm::translate(modelMatrix, renderPos);
        modelMatrix = glm::rotate(modelMatrix, angle + m_Camera.GetYawRotation() + float(M_PI / 2), glm::vec3(0, 1, 0));
        modelMatrix = glm::rotate(modelMatrix, float(M_PI / 2.f), glm::vec3(1, 0, 0));
        modelMatrix = glm::scale(modelMatrix, glm::vec3(0.05f));
        RenderArrowMesh(meshes[m_GuidingArrowMesh], shaders[m_GuidingArrowShaderName], modelMatrix);
    }
}

void DroneGames::SpawnNewCargoPosition()
{
	float terrainOffset = m_Terrain.GetLength() / 2.f;

    while (true)
    {
	    m_CargoCollectPosition = glm::vec3(
		    float(rand()) / float(RAND_MAX) * m_Terrain.GetLength() - terrainOffset,
            float(rand()) / float(RAND_MAX) * 2 + m_Terrain.GetMaxHeight(),
            float(rand()) / float(RAND_MAX) * m_Terrain.GetLength() - terrainOffset
	    );
            
		bool isInsideTree = false;
        for (const auto& treePos : m_Terrain.GetTreesPositions())
        {
			// Check if the cargo is inside a tree
			float partialLength = m_Terrain.GetLength() / (m_Terrain.GetResolution() - 1);
			glm::vec3 treePosition = -glm::vec3(treePos.x, 0, treePos.y) * partialLength + glm::vec3(terrainOffset, 0, terrainOffset);
            if (glm::distance(glm::vec3(m_CargoCollectPosition.x, 0, m_CargoCollectPosition.z), treePosition) < 2.f)
            {
				isInsideTree = true;
				break;
            }
        }
		if (isInsideTree)
			continue;

        // verify if it's inside a bunker
		bool isInsideBunker = false;
        for (const auto& bunkerPos : m_BunkersPositions)
        {
			float partialLength = m_Terrain.GetLength() / (m_Terrain.GetResolution() - 1);
            glm::vec3 bunkerPosition = -glm::vec3(bunkerPos.x, 0, bunkerPos.y) * partialLength + glm::vec3(terrainOffset, 0, terrainOffset);
            if (glm::distance(glm::vec3(m_CargoCollectPosition.x, 0, m_CargoCollectPosition.z), bunkerPosition) < m_BunkerSize.x)
            {
                isInsideBunker = true;
                break;
            }
        }

		if (!isInsideBunker)
			break;
    }
}

void DroneGames::RenderMesh(Mesh* mesh, Shader* shader, const glm::mat4& modelMatrix)
{
    if (!mesh || !shader || !shader->program)
        return;

    // Render an object using the specified shader and the specified position
    shader->Use();
    glUniformMatrix4fv(shader->loc_view_matrix, 1, GL_FALSE, glm::value_ptr(m_Camera.GetViewMatrix()));
    glUniformMatrix4fv(shader->loc_projection_matrix, 1, GL_FALSE, glm::value_ptr(m_ProjectionMatrix));
    glUniformMatrix4fv(shader->loc_model_matrix, 1, GL_FALSE, glm::value_ptr(modelMatrix));
    mesh->Render();
}

void DroneGames::RenderArrowMesh(Mesh* mesh, Shader* shader, const glm::mat4& modelMatrix)
{
    if (!mesh || !shader || !shader->program)
        return;

    // Render an object using the specified shader and the specified position
    shader->Use();
    glUniformMatrix4fv(shader->loc_view_matrix, 1, GL_FALSE, glm::value_ptr(m_Camera.GetViewMatrix()));
    glUniformMatrix4fv(shader->loc_projection_matrix, 1, GL_FALSE, glm::value_ptr(m_ProjectionMatrix));
    glUniformMatrix4fv(shader->loc_model_matrix, 1, GL_FALSE, glm::value_ptr(modelMatrix));
    // rotation
	glUniform1f(glGetUniformLocation(shader->program, "rotation"), m_CalculateArrowRotation());
    mesh->Render();
}

void DroneGames::RenderTerrainMesh(Mesh* mesh, Shader* shader, const glm::mat4& modelMatrix)
{
    if (!mesh || !shader || !shader->program)
        return;

    // Render an object using the specified shader and the specified position
    shader->Use();
    glUniformMatrix4fv(shader->loc_view_matrix, 1, GL_FALSE, glm::value_ptr(m_Camera.GetViewMatrix()));
    glUniformMatrix4fv(shader->loc_projection_matrix, 1, GL_FALSE, glm::value_ptr(m_ProjectionMatrix));
    glUniformMatrix4fv(shader->loc_model_matrix, 1, GL_FALSE, glm::value_ptr(modelMatrix));
    // Vertex shader
	glUniform2f(glGetUniformLocation(shader->program, "NoiseOffset"), m_Terrain.GetNoiseOffset().x, m_Terrain.GetNoiseOffset().y);
	glUniform1f(glGetUniformLocation(shader->program, "NoiseScalar"), m_Terrain.GetNoiseScalar());
	glUniform1f(glGetUniformLocation(shader->program, "HeightScalar"), m_Terrain.GetHeightScalar());
    glUniform1f(glGetUniformLocation(shader->program, "MaxHeight"), m_Terrain.GetMaxHeight());
    glUniform1f(glGetUniformLocation(shader->program, "MinHeight"), m_Terrain.GetMinHeight());
    // Frag shader
	glUniform2f(glGetUniformLocation(shader->program, "targetPosition"), m_TargetPosition.x, m_TargetPosition.y);
	glUniform1f(glGetUniformLocation(shader->program, "targetRadius"), m_TargetRadius);
    mesh->Render();
}

Mesh* DroneGames::CreateSquare(const std::string& name, glm::vec3 leftBottomCorner, float length, glm::vec3 color, bool fill)
{
    glm::vec3 corner = leftBottomCorner;

    std::vector<VertexFormat> vertices =
    {
        VertexFormat(corner, color),
        VertexFormat(corner + glm::vec3(length, 0, 0), color),
        VertexFormat(corner + glm::vec3(length, length, 0), color),
        VertexFormat(corner + glm::vec3(0, length, 0), color)
    };

    Mesh* square = new Mesh(name);
    std::vector<unsigned int> indices = { 0, 1, 2, 3 };

    if (!fill) {
        square->SetDrawMode(GL_LINE_LOOP);
    }
    else {
        // Draw 2 triangles. Add the remaining 2 indices
        indices.push_back(0);
        indices.push_back(2);
    }

    square->InitFromData(vertices, indices);
    return square;
}

Mesh& m1::DroneGames::CreateRectangle3D(const std::string& name, glm::vec3 leftBottomCorner, float length, float width, float height, glm::vec3 color, bool fill)
{
    glm::vec3 corner = leftBottomCorner;
	glm::vec3 lengthVector = glm::vec3(length, 0, 0);
	glm::vec3 widthVector = glm::vec3(0, 0, width);
    glm::vec3 heightVector = glm::vec3(0, height, 0);

    std::vector<VertexFormat> vertices =
    {
        // Bottom 4
        VertexFormat(corner, color),
        VertexFormat(corner + lengthVector, color),
        VertexFormat(corner + widthVector, color),
        VertexFormat(corner + lengthVector + widthVector, color),

        // Top 4
        VertexFormat(corner + heightVector, color * 1.8f),
        VertexFormat(corner + lengthVector + heightVector , color * 1.8f),
        VertexFormat(corner + widthVector + heightVector, color * 1.8f),
        VertexFormat(corner + lengthVector + widthVector + heightVector, color * 1.8f)
    };

    Mesh* rectangle = new Mesh(name);
    std::vector<unsigned int> indices =
    {
        0, 1, 2,
		1, 3, 2,

		4, 5, 6,
		5, 7, 6,

		0, 1, 4,
		1, 5, 4,

		2, 3, 6,
		3, 7, 6,

		0, 2, 4,
		2, 6, 4,

		1, 3, 5,
		3, 7, 5
	};

    if (!fill) {
        rectangle->SetDrawMode(GL_LINE_LOOP);
    }
    else {
        // Draw 2 triangles. Add the remaining 2 indices
        indices.push_back(0);
        indices.push_back(2);
    }

    rectangle->InitFromData(vertices, indices);
	AddMeshToList(rectangle);
    return *meshes[name];
}

void DroneGames::m_HandleDroneInputs(float deltaTime, int mods)
{
    glm::ivec2 changedTilt = { 0, 0 };

    if (window->KeyHold(GLFW_KEY_UP))
    {
        // Pitch camera forward
        float speed = deltaTime * m_DronePitchSpeed;
        m_Camera.RotateThirdPerson_OX(-speed);

        // Clamp pitch
        if (m_Camera.GetForwardVector().y < -abs(m_DroneMaxPitchY))
        {
            m_Camera.RotateThirdPerson_OX(speed);
        }
    }

    if (window->KeyHold(GLFW_KEY_DOWN))
    {
        // Pitch camera forward
        float speed = deltaTime * m_DronePitchSpeed;
        m_Camera.RotateThirdPerson_OX(speed);

        // Clamp pitch
        if (m_Camera.GetForwardVector().y > abs(m_DroneMaxPitchY))
        {
            m_Camera.RotateThirdPerson_OX(-speed);
        }
    }

    if (window->KeyHold(GLFW_KEY_W))
    {
        // Tilt the camera forward
        m_DroneTiltAngle.y += deltaTime * m_DroneTiltSpeed;
        m_DroneTiltAngle.y = dg::ClampMax(m_DroneTiltAngle.y, m_DroneMaxTiltAngle);
        changedTilt.y = 1;
    }

    if (window->KeyHold(GLFW_KEY_S))
    {
        // Translate the camera backwards
        m_DroneTiltAngle.y -= deltaTime * m_DroneTiltSpeed;
        m_DroneTiltAngle.y = dg::ClampMin(m_DroneTiltAngle.y, -m_DroneMaxTiltAngle);
        changedTilt.y = 1;
    }
    if (window->KeyHold(GLFW_KEY_A))
    {
        // Tilt the drone to the left
        m_DroneTiltAngle.x += deltaTime * m_DroneTiltSpeed;
        m_DroneTiltAngle.x = dg::ClampMax(m_DroneTiltAngle.x, m_DroneMaxTiltAngle);
        changedTilt.x = 1;
    }
    if (window->KeyHold(GLFW_KEY_D))
    {
        // Tilt the drone to the right
        m_DroneTiltAngle.x -= deltaTime * m_DroneTiltSpeed;
        m_DroneTiltAngle.x = dg::ClampMin(m_DroneTiltAngle.x, -m_DroneMaxTiltAngle);
        changedTilt.x = 1;
    }
    if (window->KeyHold(GLFW_KEY_SPACE))
    {
        // Thrust the drone upwards
        m_DroneAcceleration = m_DroneThrustPower;
    }
    else
    {
		m_DroneAcceleration = 0;
    }

    // Tilt decay
    glm::vec2 decay = glm::vec2
    (
        (1 - changedTilt.x) * glm::sign(m_DroneTiltAngle.x) * m_DroneTiltDecay * deltaTime,
        (1 - changedTilt.y) * glm::sign(m_DroneTiltAngle.y) * m_DroneTiltDecay * deltaTime
    );
    m_DroneTiltAngle -= decay;
}

void DroneGames::m_HandleTerrainEditorInputs(float deltaTime, int mods)
{
    // Terrain resolution
    if (window->KeyHold(GLFW_KEY_R))
    {
        float speed = (window->KeyHold(GLFW_KEY_LEFT_SHIFT)) ? -1 : 1;
        speed *= m_TerrainResolutionChangeSpeed;
        m_TerrainResolution += speed * deltaTime;
		m_TerrainResolution = dg::ClampMin(m_TerrainResolution, 2.f);

        // Regenerate terrain
        m_Terrain.GenerateTerrain((size_t)m_TerrainResolution);
        m_GenerateTerrainMesh();
        m_Terrain.GenerateTrees();
    }

    if (window->KeyHold(GLFW_KEY_T))
    {
        float speed = (window->KeyHold(GLFW_KEY_LEFT_SHIFT)) ? -1 : 1;
        speed *= m_TerrainNoiseOffsetSpeed;

        // Regenerate terrain
        m_Terrain.UpdateNoiseOffset(m_Terrain.GetNoiseOffset() + speed * deltaTime);
        m_GenerateTerrainMesh();
    }

    if (window->KeyHold(GLFW_KEY_Y))
    {
        float speed = (window->KeyHold(GLFW_KEY_LEFT_SHIFT)) ? -1 : 1;
        speed *= m_TerrainNoiseScalarSpeed;

        // Regenerate terrain
        m_Terrain.UpdateNoiseScalar(m_Terrain.GetNoiseScalar() + speed * deltaTime);
        m_GenerateTerrainMesh();
    }

    if (window->KeyHold(GLFW_KEY_U))
    {
        float speed = (window->KeyHold(GLFW_KEY_LEFT_SHIFT)) ? -1 : 1;
        speed *= m_TerrainHeightScalarSpeed;

        // Regenerate terrain
        m_Terrain.UpdateHeightScalar(m_Terrain.GetHeightScalar() + speed * deltaTime);
        m_GenerateTerrainMesh();
    }

    // Terrain length
    if (window->KeyHold(GLFW_KEY_L))
    {
        float speed = (window->KeyHold(GLFW_KEY_LEFT_SHIFT)) ? -1 : 1;
        speed *= m_TerrainLengthChangeSpeed;

        // Regenerate terrain
        m_Terrain.GenerateTerrain(m_Terrain.GetLength() + speed * deltaTime);
        m_GenerateTerrainMesh();
        m_Terrain.GenerateTrees();
        m_GenerateBunkers();
    }

    // Trees density
    if (window->KeyHold(GLFW_KEY_Z))
    {
        float speed = (window->KeyHold(GLFW_KEY_LEFT_SHIFT)) ? -1 : 1;
        speed *= 0.01;

        // Regenerate terrain
        m_Terrain.GenerateTrees(m_Terrain.GetTreesDensity() + speed * deltaTime);
    }

    // Bunkers density
    if (window->KeyHold(GLFW_KEY_X))
    {
        float speed = (window->KeyHold(GLFW_KEY_LEFT_SHIFT)) ? -1 : 1;
        speed *= 0.1f;
        m_BunkersDensity += speed * deltaTime;

        // Regenerate terrain
        m_GenerateBunkers();
    }
}

void DroneGames::OnInputUpdate(float deltaTime, int mods)
{
	m_HandleTerrainEditorInputs(deltaTime, mods);
}

void DroneGames::FrameStart()
{
    // Clears the color buffer (using the previously set color) and depth buffer
    glClearColor(0, 0.78, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::ivec2 resolution = window->GetResolution();
    // Sets the screen area where to draw
    glViewport(0, 0, resolution.x, resolution.y);
}

void DroneGames::FrameEnd()
{
}

void DroneGames::OnKeyPress(int key, int mods)
{
    // toggle wireframe view
	if (key == GLFW_KEY_M)
	{
		RENDER_WIREFRAME = !RENDER_WIREFRAME;
	}

    // Debug camera control
    if (key == GLFW_KEY_P)
    {
		// Unlock the cursor and make it visible
        m_IsCursorLocked = !m_IsCursorLocked;

        if (m_IsCursorLocked)
            window->HidePointer();
        else
            window->ShowPointer();
    }

    if (key == GLFW_KEY_F)
    {
        // If drone is holding cargo, drop it
        if (m_IsCargoCollected)
        {
            m_CargoFallPosition = m_Camera.GetTargetPosition() - glm::vec3(0, m_CargoRadius / 2 + 0.02f, 0);
            m_IsCargoDropped = true;
        }
    }
}

void DroneGames::OnMouseMove(int mouseX, int mouseY, int deltaX, int deltaY)
{
    if (m_IsCursorLocked)
    {
        float sensivityOX = 0.001f;
        float sensivityOY = 0.001f;

        window->CenterPointer();

        m_Camera.RotateThirdPerson_OX(-sensivityOX * deltaY);
        m_Camera.RotateThirdPerson_OY(-sensivityOY * deltaX);
    }
}

void DroneGames::m_GenerateFlatMesh(size_t resolution, float length, const char* meshName)
{
    if (resolution < 2)
        return;

	glm::vec3 colorMesh = glm::vec3(0.2, 0.2, 0.2);
	std::vector<VertexFormat> vertices;
    vertices.reserve(resolution * resolution);
    float step = length / (resolution - 1);
	for (int z = 0; z < resolution; z++) 
    {
		for (int x = 0; x < resolution; x++) 
        {
			vertices.push_back(VertexFormat(glm::vec3(x, 0, z) * step, colorMesh));
		}
	}

	std::vector<unsigned int> indices;
    for (int i = 0; i < resolution - 1; i++)
    {
        for (int j = 0; j < resolution - 1; j++)
        {
            indices.push_back(i * resolution + j);
            indices.push_back((i + 1) * resolution + j);
            indices.push_back(i * resolution + j + 1);

            indices.push_back((i + 1) * resolution + j);
            indices.push_back((i + 1) * resolution + j + 1);
            indices.push_back(i * resolution + j + 1);
        }
    }

    Mesh* mesh;
    if (meshes.find(meshName) == meshes.end())
    {
	    mesh = new Mesh(meshName);
	    // Create a new mesh from the data
        mesh->InitFromData(vertices, indices);
		meshes[meshName] = mesh;
    }
	else
	{
		mesh = meshes[meshName];
        mesh->ClearData();
		mesh->InitFromData(vertices, indices);
	}
}

void DroneGames::m_GenerateTerrainMesh()
{
    size_t resolution = m_Terrain.GetResolution();
    float length = m_Terrain.GetLength();
	m_GenerateFlatMesh(resolution, length, m_TerrainMeshName);
}

void DroneGames::m_GenerateBunkers()
{
    m_BunkersPositions.clear();

    size_t bunkerMaxSize = dg::Max(m_BunkerSize.x, dg::Max(m_BunkerSize.y, m_BunkerSize.z));
    size_t resolution = m_Terrain.GetResolution();
    float partialLength = m_Terrain.GetLength() / float(resolution - 1);
    for (size_t z = bunkerMaxSize / partialLength; z < resolution; z += 10)
        for (size_t x = bunkerMaxSize / partialLength; x < resolution; x += 10)
            if (dg::Random(glm::vec2(x, z) * partialLength) < m_BunkersDensity)
            {
                // check if tree is within some threshold to any other tree
                bool isClose = false;
                for (const auto& otherBunker : m_BunkersPositions)
                {
                    float distance = glm::distance(glm::vec2(x, z), otherBunker);
                    if (distance < 1.5 * bunkerMaxSize)
                    {
                        isClose = true;
                        break;
                    }
                }

                if (!isClose)
                    m_BunkersPositions.push_back(glm::vec2(x, z));
            }
}

void DroneGames::OnMouseBtnPress(int mouseX, int mouseY, int button, int mods)
{
}

void DroneGames::OnMouseBtnRelease(int mouseX, int mouseY, int button, int mods)
{
}

void DroneGames::OnMouseScroll(int mouseX, int mouseY, int offsetX, int offsetY)
{
}

void DroneGames::OnWindowResize(int width, int height)
{
}
