//-----------------------------------------------------------------------------
// File : MeshLoader.cpp
// Desc : Mesh Loader.
// Copyright(c) Project Asura. All right reserved.
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <MeshLoader.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>
#include <codecvt>
#include <cassert>
#include <meshoptimizer.h>
#include <asdxHash.h>


//-----------------------------------------------------------------------------
//      コンストラクタです.
//-----------------------------------------------------------------------------
MeshLoader::MeshLoader()
: m_pScene(nullptr)
{ /* DO_NOTHING */ }

//-----------------------------------------------------------------------------
//      デストラクタです.
//-----------------------------------------------------------------------------
MeshLoader::~MeshLoader()
{ /* DO_NOTHING */ }

//-----------------------------------------------------------------------------
//      メッシュをロードします.
//-----------------------------------------------------------------------------
bool MeshLoader::Load(const char* filename, asdx::ResModel& model)
{
    if (filename == nullptr)
    { return false; }

    Assimp::Importer importer;
    unsigned int flag = 0;
    flag |= aiProcess_Triangulate;
    flag |= aiProcess_PreTransformVertices;
    flag |= aiProcess_CalcTangentSpace;
    flag |= aiProcess_GenSmoothNormals;
    flag |= aiProcess_GenUVCoords;
    flag |= aiProcess_RemoveRedundantMaterials;
    flag |= aiProcess_OptimizeMeshes;

    // ファイルを読み込み.
    m_pScene = importer.ReadFile(filename, flag);

    // チェック.
    if (m_pScene == nullptr)
    { return false; }

    // メッシュデータを変換.
    for(auto i=0u; i<m_pScene->mNumMeshes; ++i)
    {
        const aiMesh* pMesh = m_pScene->mMeshes[i];
        ParseMesh(model, pMesh);
    }

    // 不要になったのでクリア.
    importer.FreeScene();
    m_pScene = nullptr;

    // 正常終了.
    return true;
}

//-----------------------------------------------------------------------------
//      静的メッシュデータを解析します.
//-----------------------------------------------------------------------------
void MeshLoader::ParseMesh(asdx::ResModel& model, const aiMesh* pSrcMesh)
{
    // マテリアル番号を設定.
    auto matId = pSrcMesh->mMaterialIndex;
    uint32_t matHash = matId;

    // マテリアルハッシュ生成.
    {
        auto pMaterial = m_pScene->mMaterials[matId];
        aiString matName;
        if (pMaterial->Get(AI_MATKEY_NAME, matName) == AI_SUCCESS)
        { matHash = asdx::Fnv1a(matName.C_Str()).GetHash(); }
    }

    asdx::ResMesh dstMesh;
    dstMesh.MeshHash        = asdx::Fnv1a(pSrcMesh->mName.C_Str()).GetHash();
    dstMesh.MatrerialHash   = matHash;


    aiVector3D zero3D(0.0f, 0.0f, 0.0f);
    aiColor4D  white (1.0f, 1.0f, 1.0f, 1.0f);

    // 頂点データのメモリを確保.
    dstMesh.Positions.resize(pSrcMesh->mNumVertices);
    if (pSrcMesh->HasNormals())
    { dstMesh.TangentSpaces.resize(pSrcMesh->mNumVertices); }

    for(auto i=0; i<4; ++i)
    {
        if (pSrcMesh->HasTextureCoords(i))
        { dstMesh.TexCoords[i].resize(pSrcMesh->mNumVertices); }
    }

    if (pSrcMesh->HasVertexColors(0))
    { dstMesh.Colors.resize(pSrcMesh->mNumVertices); }

    if (pSrcMesh->HasBones())
    {
        dstMesh.BoneIndices.resize(pSrcMesh->mNumVertices);
        dstMesh.BoneWeights.resize(pSrcMesh->mNumVertices);
    }

    for(auto i=0u; i<pSrcMesh->mNumVertices; ++i)
    {
        auto pPosition  = &(pSrcMesh->mVertices[i]);

        if (pSrcMesh->HasNormals() && pSrcMesh->HasTangentsAndBitangents())
        {
            auto pNormal = &(pSrcMesh->mNormals[i]);
            auto pTangent = &(pSrcMesh->mTangents[i]);

            auto N = asdx::Vector3(pNormal->x, pNormal->y, pNormal->z);
            auto T = asdx::Vector3(pTangent->x, pTangent->y, pTangent->z);

            dstMesh.TangentSpaces[i] = EncodeTBN(N, T, 0);
        }

        if (pSrcMesh->HasNormals() && !pSrcMesh->HasTangentsAndBitangents())
        {
            auto pNormal = &(pSrcMesh->mNormals[i]);
            auto N = asdx::Vector3(pNormal->x, pNormal->y, pNormal->z);
            asdx::Vector3 T, B;
            asdx::CalcONB(N, T, B);

            dstMesh.TangentSpaces[i] = EncodeTBN(N, T, 0);
        }

        for(auto j=0; j<4; ++j)
        {
            if (pSrcMesh->HasTextureCoords(j))
            {
                auto pTexCorod = &(pSrcMesh->mTextureCoords[j][i]);
                dstMesh.TexCoords[j][i] = asdx::EncodeTexCoord(asdx::Vector2(pTexCorod->x, pTexCorod->y));
            }
        }

        if (pSrcMesh->HasVertexColors(0))
        {
            auto pColor = &(pSrcMesh->mColors[0][i]);
            dstMesh.Colors[i] = asdx::ToUnorm(asdx::Vector4(pColor->r, pColor->g, pColor->b, pColor->a));
        }

        if (pSrcMesh->HasBones())
        {
            dstMesh.BoneIndices[i] = asdx::ResBoneIndex(0, 0, 0, 0);
            dstMesh.BoneWeights[i] = asdx::Vector4(0.0f, 0.0f, 0.0f, 0.0f);
        }
    }

    // ボーン番号と重みを設定する.
    for(auto i=0u; i<pSrcMesh->mNumBones; ++i)
    {
        for(auto j=0u; j<pSrcMesh->mBones[i]->mNumWeights; ++j)
        {
            auto weight     = pSrcMesh->mBones[i]->mWeights[j];
            auto srcBoneIndex  = uint16_t(i);
            auto srcBoneWeight = weight.mWeight;

            auto& dstBoneIndex = dstMesh.BoneIndices[weight.mVertexId];
            auto& dstBoneWeight = dstMesh.BoneWeights[weight.mVertexId];
            auto detect = false;

            if (dstBoneWeight.x == 0.0f)
            {
                dstBoneIndex.Index0 = srcBoneIndex;
                dstBoneWeight.x     = srcBoneWeight;
                detect = true;
            }
            else if (dstBoneWeight.y == 0.0f)
            {
                dstBoneIndex.Index1 = srcBoneIndex;
                dstBoneWeight.y     = srcBoneWeight;
                detect = true;
            }
            else if (dstBoneWeight.z == 0.0f)
            {
                dstBoneIndex.Index2 = srcBoneIndex;
                dstBoneWeight.z     = srcBoneWeight;
                detect = true;
            }
            else if (dstBoneWeight.w == 0.0f)
            {
                dstBoneIndex.Index3 = srcBoneIndex;
                dstBoneWeight.z     = srcBoneWeight;
                detect = true;
            }

            // いっぱいだった場合は，重みが一番小さい所に入れる.
            if (!detect)
            {
                auto mini = dstBoneWeight.x;
                auto idx = 0;
                if (mini > dstBoneWeight.y)
                {
                    mini = dstBoneWeight.y;
                    idx = 1;
                }
                if (mini > dstBoneWeight.z)
                {
                    mini = dstBoneWeight.z;
                    idx = 2;
                }
                if (mini > dstBoneWeight.w)
                {
                    mini = dstBoneWeight.w;
                    idx = 3;
                }

                // 入力データの方が重みが小さい場合は処理しない.
                if (mini > srcBoneWeight)
                { continue; }

                if (idx == 0)
                {
                    dstBoneIndex.Index0 = srcBoneIndex;
                    dstBoneWeight.x     = srcBoneWeight;
                }
                else if (idx == 1)
                {
                    dstBoneIndex.Index1 = srcBoneIndex;
                    dstBoneWeight.y     = srcBoneWeight;
                }
                else if (idx == 2)
                {
                    dstBoneIndex.Index2 = srcBoneIndex;
                    dstBoneWeight.z     = srcBoneWeight;
                }
                else if (idx == 3)
                {
                    dstBoneIndex.Index3 = srcBoneIndex;
                    dstBoneWeight.w     = srcBoneWeight;
                }
            }
        }
    }

    // 頂点インデックスのメモリを確保.
    std::vector<uint32_t>   vertexIndices;
    vertexIndices.resize(pSrcMesh->mNumFaces * 3);

    for(auto i=0u; i<pSrcMesh->mNumFaces; ++i)
    {
        const auto& face = pSrcMesh->mFaces[i];
        assert(face.mNumIndices == 3);  // 三角形化しているので必ず3になっている.

        vertexIndices[i * 3 + 0] = face.mIndices[0];
        vertexIndices[i * 3 + 1] = face.mIndices[1];
        vertexIndices[i * 3 + 2] = face.mIndices[2];
    }

    // 最適化.
    {
        std::vector<uint32_t> remap(vertexIndices.size());

        // 重複データを削除するための再マッピング用インデックスを生成.
        meshopt_Stream streams[9];
        streams[0].data     = dstMesh.Positions.data();
        streams[0].size     = sizeof(dstMesh.Positions[0]) * dstMesh.Positions.size();
        streams[0].stride   = sizeof(dstMesh.Positions[0]);

        auto idx = 1;
        if (!dstMesh.TangentSpaces.empty())
        {
            streams[idx].data   = dstMesh.TangentSpaces.data();
            streams[idx].size   = sizeof(dstMesh.TangentSpaces[0]) * dstMesh.TangentSpaces.size();
            streams[idx].stride = sizeof(dstMesh.TangentSpaces[0]);
            idx++;
        }

        if (!dstMesh.Colors.empty())
        {
            streams[idx].data   = dstMesh.Colors.data();
            streams[idx].size   = sizeof(dstMesh.Colors[0]) * dstMesh.Colors.size();
            streams[idx].stride = sizeof(dstMesh.Colors[0]);
            idx++;
        }

        for(auto i=0; i<4; ++i)
        {
            if (!dstMesh.TexCoords[i].empty())
            {
                streams[idx].data   = dstMesh.TexCoords[i].data();
                streams[idx].size   = sizeof(dstMesh.TexCoords[i][0]) * dstMesh.TexCoords[i].size();
                streams[idx].stride = sizeof(dstMesh.TexCoords[i][0]);
                idx++;
            }
        }

        if (!dstMesh.BoneIndices.empty())
        {
            streams[idx].data   = dstMesh.BoneIndices.data();
            streams[idx].size   = sizeof(dstMesh.BoneIndices[0]) * dstMesh.BoneIndices.size();
            streams[idx].stride = sizeof(dstMesh.BoneIndices[0]);
            idx++;
        }

        if (!dstMesh.BoneWeights.empty())
        {
            streams[idx].data   = dstMesh.BoneWeights.data();
            streams[idx].size   = sizeof(dstMesh.BoneWeights[0]) * dstMesh.BoneWeights.size();
            streams[idx].stride = sizeof(dstMesh.BoneWeights[0]);
            idx++;
        }

        auto vertexCount = meshopt_generateVertexRemapMulti(
            remap.data(),
            vertexIndices.data(),
            vertexIndices.size(),
            dstMesh.Positions.size(),
            streams,
            idx
        );

        std::vector<uint32_t> indices(vertexIndices.size());

        // 頂点インデックスを再マッピング.
        meshopt_remapIndexBuffer(
            indices.data(),
            vertexIndices.data(),
            vertexIndices.size(),
            remap.data());

        // 頂点キャッシュ最適化.
        meshopt_optimizeVertexCache(
            vertexIndices.data(),
            indices.data(),
            indices.size(),
            vertexCount);

        // 不要になったメモリを解放.
        indices.clear();
        indices.shrink_to_fit();

        meshopt_optimizeVertexFetchRemap(
            vertexIndices.data(),
            vertexIndices.data(),
            vertexIndices.size(),
            vertexCount);

        {
            // 頂点データを再マッピング.
            meshopt_remapVertexBuffer(
                dstMesh.Positions.data(),
                dstMesh.Positions.data(),
                dstMesh.Positions.size(),
                sizeof(dstMesh.Positions[0]),
                remap.data());

            dstMesh.Positions.resize(vertexCount);
            dstMesh.Positions.shrink_to_fit();
        }

        if (!dstMesh.TangentSpaces.empty())
        {
            meshopt_remapVertexBuffer(
                dstMesh.TangentSpaces.data(),
                dstMesh.TangentSpaces.data(),
                dstMesh.TangentSpaces.size(),
                sizeof(dstMesh.TangentSpaces[0]),
                remap.data());

            dstMesh.TangentSpaces.resize(vertexCount);
            dstMesh.TangentSpaces.shrink_to_fit();
        }

        if (!dstMesh.Colors.empty())
        {
            meshopt_remapVertexBuffer(
                dstMesh.Colors.data(),
                dstMesh.Colors.data(),
                dstMesh.Colors.size(),
                sizeof(dstMesh.Colors[0]),
                remap.data());

            dstMesh.Colors.resize(vertexCount);
            dstMesh.Colors.shrink_to_fit();
        }

        for(auto i=0; i<4; ++i)
        {
            if (!dstMesh.TexCoords[i].empty())
            {
                meshopt_remapVertexBuffer(
                    dstMesh.TexCoords[i].data(),
                    dstMesh.TexCoords[i].data(),
                    dstMesh.TexCoords[i].size(),
                    sizeof(dstMesh.TexCoords[i][0]),
                    remap.data());

                dstMesh.TexCoords[i].resize(vertexCount);
                dstMesh.TexCoords[i].shrink_to_fit();
            }
        }

        if (!dstMesh.BoneIndices.empty())
        {
            meshopt_remapVertexBuffer(
                dstMesh.BoneIndices.data(),
                dstMesh.BoneIndices.data(),
                dstMesh.BoneIndices.size(),
                sizeof(dstMesh.BoneIndices[0]),
                remap.data());

            dstMesh.BoneIndices.resize(vertexCount);
            dstMesh.BoneIndices.shrink_to_fit();
        }

        if (!dstMesh.BoneWeights.empty())
        {
            meshopt_remapVertexBuffer(
                dstMesh.BoneWeights.data(),
                dstMesh.BoneWeights.data(),
                dstMesh.BoneWeights.size(),
                sizeof(dstMesh.BoneWeights[0]),
                remap.data());

            dstMesh.BoneWeights.resize(vertexCount);
            dstMesh.BoneWeights.shrink_to_fit();
        }

        // 不要になったメモリを解放.
        remap.clear();
        remap.shrink_to_fit();
    }

    // メッシュレット生成.
    {
        // see. https://developer.nvidia.com/blog/introduction-turing-mesh-shaders/
        const size_t kMaxVertices   = 64;
        const size_t kMaxPrimitives = 126;

        std::vector<meshopt_Meshlet> meshlets(
            meshopt_buildMeshletsBound(
                vertexIndices.size(),
                kMaxVertices,
                kMaxPrimitives));
        meshlets.resize(
            meshopt_buildMeshlets(
                meshlets.data(),
                vertexIndices.data(),
                vertexIndices.size(),
                dstMesh.Positions.size(),
                kMaxVertices,
                kMaxPrimitives));

        // 最大値でメモリを予約.
        dstMesh.Indices     .reserve(meshlets.size() * kMaxVertices);
        dstMesh.Primitives  .reserve(meshlets.size() * kMaxPrimitives);

        for(auto& meshlet : meshlets)
        {
            auto vertexOffset    = uint32_t(dstMesh.Indices     .size());
            auto primitiveOffset = uint32_t(dstMesh.Primitives  .size());

            for(auto i=0u; i<meshlet.vertex_count; ++i)
            { dstMesh.Indices.push_back(meshlet.vertices[i]); }

            for(auto i=0; i<meshlet.triangle_count; ++i)
            {
                asdx::ResPrimitive tris = {};
                tris.Index1 = meshlet.indices[i][0];
                tris.Index0 = meshlet.indices[i][1];
                tris.Index2 = meshlet.indices[i][2];
                dstMesh.Primitives.push_back(tris);
            }

            auto bounds = meshopt_computeMeshletBounds(
                &meshlet, 
                &dstMesh.Positions[0].x,
                dstMesh.Positions.size(),
                sizeof(dstMesh.Positions[0]));

            // メッシュレットデータ設定.
            asdx::ResMeshlet m = {};
            m.VertexCount       = meshlet.vertex_count;
            m.VertexOffset      = vertexOffset;
            m.PrimitiveCount    = meshlet.triangle_count;
            m.PrimitiveOffset   = primitiveOffset;

            dstMesh.Meshlets.push_back(m);

            // カリングデータ設定.
            auto sinAngle = sqrt(1.0f - bounds.cone_cutoff * bounds.cone_cutoff);
            auto normalCone = asdx::Vector4(
                bounds.cone_axis[0] * 0.5f + 0.5f,
                bounds.cone_axis[1] * 0.5f + 0.5f,
                bounds.cone_axis[2] * 0.5f + 0.5f,
                sinAngle * 0.5f + 0.5f);

            asdx::ResCullingInfo c = {};
            c.BoundingSphere = asdx::Vector4(bounds.center[0], bounds.center[1], bounds.center[2], bounds.radius);
            c.NormalCone     = asdx::ToUnorm(normalCone);
            dstMesh.CullingInfos.push_back(c);
        }

        // サイズ最適化.
        dstMesh.Indices     .shrink_to_fit();
        dstMesh.Primitives  .shrink_to_fit();
        dstMesh.Meshlets    .shrink_to_fit();
        dstMesh.CullingInfos.shrink_to_fit();
    }

    model.Meshes.push_back(dstMesh);
}
