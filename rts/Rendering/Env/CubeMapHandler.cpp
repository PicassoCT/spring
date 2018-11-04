/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#include "Game/Camera.h"
#include "Game/CameraHandler.h"
#include "Game/Game.h"
#include "Map/BaseGroundDrawer.h"
#include "Map/Ground.h"
#include "Map/ReadMap.h"
#include "Map/MapInfo.h"
#include "Rendering/GlobalRendering.h"
#include "Rendering/GL/myGL.h"
#include "Rendering/Env/ISky.h"
#include "Rendering/Env/SunLighting.h"
#include "Rendering/Env/CubeMapHandler.h"
#include "System/Config/ConfigHandler.h"

CONFIG(int, CubeTexSizeReflection).defaultValue(128).minimumValue(1);
CONFIG(bool, CubeTexGenerateMipMaps).defaultValue(false);

CubeMapHandler cubeMapHandler;

bool CubeMapHandler::Init() {
	envReflectionTexID = 0;
	skyReflectionTexID = 0;

	reflectionTexSize = configHandler->GetInt("CubeTexSizeReflection");
	currReflectionFace = 0;

	generateMipMaps = configHandler->GetBool("CubeTexGenerateMipMaps");

	{
		glGenTextures(1, &envReflectionTexID);
		glBindTexture(GL_TEXTURE_CUBE_MAP, envReflectionTexID);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, generateMipMaps? GL_LINEAR_MIPMAP_LINEAR: GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER,                                           GL_LINEAR); // magnification doesn't use mips
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
	}

	if (generateMipMaps)
		glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

	if ((mapSkyReflections = !mapInfo->smf.skyReflectModTexName.empty())) {
		glGenTextures(1, &skyReflectionTexID);
		glBindTexture(GL_TEXTURE_CUBE_MAP, skyReflectionTexID);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGBA8, reflectionTexSize, reflectionTexSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
	}


	// reflectionCubeFBO is no-op constructed, has to be initialized manually
	reflectionCubeFBO.Init(false);

	if (reflectionCubeFBO.IsValid()) {
		reflectionCubeFBO.Bind();
		reflectionCubeFBO.CreateRenderBuffer(GL_DEPTH_ATTACHMENT, GL_DEPTH_COMPONENT, reflectionTexSize, reflectionTexSize);
		reflectionCubeFBO.Unbind();
		return true;
	}

	Free();
	return false;
}

void CubeMapHandler::Free() {
	if (envReflectionTexID != 0) {
		glDeleteTextures(1, &envReflectionTexID);
		envReflectionTexID = 0;
	}
	if (skyReflectionTexID != 0) {
		glDeleteTextures(1, &skyReflectionTexID);
		skyReflectionTexID = 0;
	}

	reflectionCubeFBO.Kill();
}



void CubeMapHandler::UpdateReflectionTexture()
{
	// NOTE:
	//   we unbind later in WorldDrawer::GenerateIBLTextures() to save render
	//   context switches (which are one of the slowest OpenGL operations!)
	//   together with VP restoration
	reflectionCubeFBO.Bind();

	switch (currReflectionFace) {
		case 0: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_POSITIVE_X,  RgtVector, false); } break;
		case 1: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, -RgtVector, false); } break;
		case 2: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_POSITIVE_Y,  UpVector,  false); } break;
		case 3: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, -UpVector,  false); } break;
		case 4: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_POSITIVE_Z,  FwdVector, false); } break;
		case 5: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, -FwdVector, false); } break;
		default: {} break;
	}

	if (mapSkyReflections) {
		// draw only the sky (into its own cubemap) for SSMF
		// by reusing data from previous frame we could also
		// make terrain reflect itself, not just the sky
		switch (currReflectionFace) {
			case  6: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_POSITIVE_X,  RgtVector, true); } break;
			case  7: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, -RgtVector, true); } break;
			case  8: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_POSITIVE_Y,  UpVector,  true); } break;
			case  9: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, -UpVector,  true); } break;
			case 10: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_POSITIVE_Z,  FwdVector, true); } break;
			case 11: { CreateReflectionFace(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, -FwdVector, true); } break;
			default: {} break;
		}

		currReflectionFace +=  1;
		currReflectionFace %= 12;
	} else {
		// touch the FBO at least once per frame
		currReflectionFace += 1;
		currReflectionFace %= 6;
	}

	if (generateMipMaps && currReflectionFace == 0) {
		glBindTexture(GL_TEXTURE_CUBE_MAP, envReflectionTexID);
		glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
		glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
	}
}

void CubeMapHandler::CreateReflectionFace(unsigned int glType, const float3& camDir, bool skyOnly)
{
	reflectionCubeFBO.AttachTexture((skyOnly? skyReflectionTexID: envReflectionTexID), glType);

	glAttribStatePtr->PushDepthBufferBit();
	glAttribStatePtr->ClearColor(sky->fogColor[0], sky->fogColor[1], sky->fogColor[2], 1.0f);
	glAttribStatePtr->Clear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	if (!skyOnly) {
		glAttribStatePtr->EnableDepthMask();
		glAttribStatePtr->EnableDepthTest();
	} else {
		// do not need depth-testing for the sky alone
		glAttribStatePtr->DisableDepthMask();
		glAttribStatePtr->DisableDepthTest();
	}

	{
		CCamera* prvCam = CCameraHandler::GetSetActiveCamera(CCamera::CAMTYPE_ENVMAP);
		CCamera* curCam = CCameraHandler::GetActiveCamera();

		bool draw = true;

		#if 0
		switch (glType) {
			case GL_TEXTURE_CUBE_MAP_POSITIVE_X: { glAttribStatePtr->ClearColor(1.0f, 0.0f, 0.0f, 1.0f); draw = false; } break; // red
			case GL_TEXTURE_CUBE_MAP_NEGATIVE_X: { glAttribStatePtr->ClearColor(0.0f, 1.0f, 0.0f, 1.0f); draw = false; } break; // green
			case GL_TEXTURE_CUBE_MAP_POSITIVE_Y: { glAttribStatePtr->ClearColor(0.0f, 0.0f, 1.0f, 1.0f); draw = false; } break; // blue
			case GL_TEXTURE_CUBE_MAP_NEGATIVE_Y: { glAttribStatePtr->ClearColor(1.0f, 1.0f, 0.0f, 1.0f); draw = false; } break; // yellow
			case GL_TEXTURE_CUBE_MAP_POSITIVE_Z: { glAttribStatePtr->ClearColor(1.0f, 0.0f, 1.0f, 1.0f); draw = false; } break; // purple
			case GL_TEXTURE_CUBE_MAP_NEGATIVE_Z: { glAttribStatePtr->ClearColor(0.0f, 1.0f, 1.0f, 1.0f); draw = false; } break; // cyan
			default: {} break;
		}

		glAttribStatePtr->Clear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
		#endif

		#if 1
		// work around CCameraHandler::GetRgtFromRot bugs
		switch (glType) {
			case GL_TEXTURE_CUBE_MAP_POSITIVE_X: { /*fwd =  Rgt*/ curCam->right =  FwdVector; curCam->up =   UpVector; } break;
			case GL_TEXTURE_CUBE_MAP_NEGATIVE_X: { /*fwd = -Rgt*/ curCam->right = -FwdVector; curCam->up =   UpVector; } break;
			case GL_TEXTURE_CUBE_MAP_POSITIVE_Y: { /*fwd =   Up*/ curCam->right =  RgtVector; curCam->up = -FwdVector; } break;
			case GL_TEXTURE_CUBE_MAP_NEGATIVE_Y: { /*fwd =  -Up*/ curCam->right =  RgtVector; curCam->up =  FwdVector; } break;
			case GL_TEXTURE_CUBE_MAP_POSITIVE_Z: { /*fwd =  Fwd*/ curCam->right = -RgtVector; curCam->up =   UpVector; } break;
			case GL_TEXTURE_CUBE_MAP_NEGATIVE_Z: { /*fwd = -Fwd*/ curCam->right =  RgtVector; curCam->up =   UpVector; } break;
			default: {} break;
		}

		if (draw) {
			// env-reflections are only correct when drawn from an inverted
			// perspective (meaning right becomes left and up becomes down)
			curCam->forward  = camDir;
			curCam->right   *= -1.0f;
			curCam->up      *= -1.0f;

			// we want a *horizontal* FOV of 90 degrees; this gets us close
			// enough (assumes a 16:10 horizontal aspect-ratio common case)
			curCam->SetVFOV(64.0f);
			curCam->SetPos(prvCam->GetPos());
			#else
			curCam->SetRotZ(0.0f);
			curCam->SetDir(camDir);
			#endif

			curCam->UpdateLoadViewPort(0, 0, reflectionTexSize, reflectionTexSize);
			// update matrices (not dirs or viewport)
			curCam->Update(false, true, false);

			// generate the face
			game->SetDrawMode(Game::ReflectionDraw);
			sky->Draw(Game::ReflectionDraw);

			if (!skyOnly)
				readMap->GetGroundDrawer()->Draw(DrawPass::TerrainReflection);

			game->SetDrawMode(Game::NormalDraw);
		}

		CCameraHandler::SetActiveCamera(prvCam->GetCamType());
		prvCam->Update();
	}

	glAttribStatePtr->PopBits();
}

