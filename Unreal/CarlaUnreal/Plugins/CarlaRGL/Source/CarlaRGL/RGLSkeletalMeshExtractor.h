#pragma once
// Pure UE -> POD extraction for skeletal meshes (spec §3). No RGL calls, no scene state.
#ifdef WITH_RGL
#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>
#include <rgl/api/core.h>

class USkeletalMesh;
class USkinnedMeshComponent;

struct FRGLSkeletalMeshData
{
    TArray<rgl_vec3f>          Vertices;      // rest pose, component space, meters
    TArray<rgl_vec3i>          Indices;       // triangles; bDisabled sections excluded
    TArray<rgl_bone_weights_t> Weights;       // per vertex; 4 influences; RAW bone indices
    TArray<rgl_mat3x4f>        RestposesInv;  // per raw bone; inverse bind pose; RGL frame
    int32                      RawBoneNum = 0;
    int32                      LODIndex = -1;
};

struct FRGLSkeletalLODReadiness
{
    uint32 NumVertices = 0; bool bPositionsCPU = false;
    int32  IndexCount = 0;  int32 IndexBytes = 0;
    uint32 WeightVertices = 0; bool bWeightData = false; bool bVariableBones = false; bool bLookupData = false;
    int32  Sections = 0; int32 RawBoneNum = 0; int32 InvBindCount = 0;
    FString FailReason;
    FString ToString() const
    {
        return FString::Printf(TEXT("verts=%u posCPU=%d idx=%d idxBytes=%d wVerts=%u wData=%d varBones=%d lookup=%d sections=%d rawBones=%d invBind=%d %s"),
            NumVertices, bPositionsCPU, IndexCount, IndexBytes, WeightVertices, bWeightData, bVariableBones, bLookupData, Sections, RawBoneNum, InvBindCount, *FailReason);
    }
};

namespace RGLSkeletal
{
    bool  ReduceInfluences(const int32* RawBones, const uint16* Weights16, int32 Count, rgl_bone_weights_t& Out);
    bool  IsLODReadable(const USkeletalMesh* Mesh, int32 LODIndex, FRGLSkeletalLODReadiness& Out);
    int32 SelectReadableLOD(const USkeletalMesh* Mesh, FString& OutReason);
    bool  ExtractSkeletalMesh(const USkeletalMesh* Mesh, FRGLSkeletalMeshData& Out, FString& OutReason);
    // Task 4 adds: bool BuildWorldPose(const USkinnedMeshComponent*, int32 RawBoneNum, TArray<rgl_mat3x4f>&);
}
#endif // WITH_RGL
