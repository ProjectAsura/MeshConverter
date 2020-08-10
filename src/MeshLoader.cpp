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
    dstMesh.Vertices.resize(pSrcMesh->mNumVertices);

    auto hasBone = pSrcMesh->mNumBones > 0;
    if (hasBone)
    { dstMesh.SkinVertices.resize(pSrcMesh->mNumVertices); }

    for(auto i=0u; i<pSrcMesh->mNumVertices; ++i)
    {
        auto pPosition  = &(pSrcMesh->mVertices[i]);
        auto pNormal    = &(pSrcMesh->mNormals[i]);
        auto pTexCoord0 = (pSrcMesh->HasTextureCoords(0)) ? &(pSrcMesh->mTextureCoords[0][i]) : &zero3D;
        auto pTexCoord1 = (pSrcMesh->HasTextureCoords(1)) ? &(pSrcMesh->mTextureCoords[1][i]) : &zero3D;
        auto pTexCoord2 = (pSrcMesh->HasTextureCoords(2)) ? &(pSrcMesh->mTextureCoords[2][i]) : &zero3D;
        auto pTexCoord3 = (pSrcMesh->HasTextureCoords(3)) ? &(pSrcMesh->mTextureCoords[3][i]) : &zero3D;
        auto pTangent   = (pSrcMesh->HasTangentsAndBitangents()) ? &(pSrcMesh->mTangents[i])  : &zero3D;
        auto pColor     = (pSrcMesh->HasVertexColors(0)) ? &(pSrcMesh->mColors[0][i]) : &white;

        asdx::Vector3 N(pNormal ->x, pNormal ->y, pNormal ->z);
        asdx::Vector3 T(pTangent->x, pTangent->y, pTangent->z);

        dstMesh.Vertices[i].Position        = asdx::Vector3(pPosition->x, pPosition->y, pPosition->z);
        dstMesh.Vertices[i].TexCoord0       = asdx::EncodeTexCoord(asdx::Vector2(pTexCoord0->x, pTexCoord0->y));
        dstMesh.Vertices[i].TexCoord1       = asdx::EncodeTexCoord(asdx::Vector2(pTexCoord1->x, pTexCoord1->y));
        dstMesh.Vertices[i].TexCoord2       = asdx::EncodeTexCoord(asdx::Vector2(pTexCoord2->x, pTexCoord2->y));
        dstMesh.Vertices[i].TexCoord3       = asdx::EncodeTexCoord(asdx::Vector2(pTexCoord3->x, pTexCoord3->y));
        dstMesh.Vertices[i].TangentSpace    = asdx::EncodeTBN(N, T, 0);
        dstMesh.Vertices[i].Color           = asdx::ToUnorm(asdx::Vector4(pColor->r, pColor->g, pColor->b, pColor->a));

        if (hasBone)
        {
            dstMesh.SkinVertices[i].BoneIndex       = asdx::ResBoneIndex(0, 0, 0, 0);           // 初期化.
            dstMesh.SkinVertices[i].BoneWeights     = asdx::Vector4(0.0f, 0.0f, 0.0f, 0.0f);    // 初期化.
        }
    }

    // ボーン番号と重みを設定する.
    for(auto i=0u; i<pSrcMesh->mNumBones; ++i)
    {
        for(auto j=0u; j<pSrcMesh->mBones[i]->mNumWeights; ++j)
        {
            auto weight     = pSrcMesh->mBones[i]->mWeights[j];
            auto boneIndex  = uint16_t(i);
            auto boneWeight = weight.mWeight;

            auto& vertex = dstMesh.SkinVertices[weight.mVertexId];
            auto processed = false;

            if (vertex.BoneWeights.x == 0.0f)
            {
                vertex.BoneIndex.Index0 = boneIndex;
                vertex.BoneWeights.x    = boneWeight;
                processed = true;
            }
            else if (vertex.BoneWeights.y == 0.0f)
            {
                vertex.BoneIndex.Index1 = boneIndex;
                vertex.BoneWeights.y    = boneWeight;
                processed = true;
            }
            else if (vertex.BoneWeights.z == 0.0f)
            {
                vertex.BoneIndex.Index2 = boneIndex;
                vertex.BoneWeights.z    = boneWeight;
                processed = true;
            }
            else if (vertex.BoneWeights.w == 0.0f)
            {
                vertex.BoneIndex.Index3 = boneIndex;
                vertex.BoneWeights.z    = boneWeight;
                processed = true;
            }

            if (!processed)
            {
                auto mini = vertex.BoneWeights.x;
                auto idx = 0;
                if (mini > vertex.BoneWeights.y)
                {
                    mini = vertex.BoneWeights.y;
                    idx = 1;
                }
                if (mini > vertex.BoneWeights.z)
                {
                    mini = vertex.BoneWeights.z;
                    idx = 2;
                }
                if (mini > vertex.BoneWeights.w)
                {
                    mini = vertex.BoneWeights.w;
                    idx = 3;
                }

                if (mini > boneWeight)
                { continue; }

                if (idx == 0)
                {
                    vertex.BoneIndex.Index0 = boneIndex;
                    vertex.BoneWeights.x    = boneWeight;
                }
                else if (idx == 1)
                {
                    vertex.BoneIndex.Index1 = boneIndex;
                    vertex.BoneWeights.y    = boneWeight;
                }
                else if (idx == 2)
                {
                    vertex.BoneIndex.Index2 = boneIndex;
                    vertex.BoneWeights.z    = boneWeight;
                }
                else if (idx == 3)
                {
                    vertex.BoneIndex.Index3 = boneIndex;
                    vertex.BoneWeights.w    = boneWeight;
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
        size_t vertexCount;
        if (hasBone)
        {
            meshopt_Stream streams[2];
            streams[0].data     = dstMesh.Vertices.data();
            streams[0].size     = dstMesh.Vertices.size() * sizeof(dstMesh.Vertices[0]);
            streams[0].stride   = sizeof(dstMesh.Vertices[0]);

            streams[1].data     = dstMesh.SkinVertices.data();
            streams[1].size     = dstMesh.SkinVertices.size() * sizeof(dstMesh.SkinVertices[0]);
            streams[1].stride   = sizeof(dstMesh.SkinVertices[0]);

            vertexCount = meshopt_generateVertexRemapMulti(
                remap.data(),
                vertexIndices.data(),
                vertexIndices.size(),
                dstMesh.Vertices.size(),
                streams,
                2
            );

            std::vector<asdx::ResMeshVertex> vertices(vertexCount);
            std::vector<asdx::ResMeshSkinVertex> skinVertices(vertexCount);
            std::vector<uint32_t> indices(vertexIndices.size());

            // 頂点インデックスを再マッピング.
            meshopt_remapIndexBuffer(
                indices.data(),
                vertexIndices.data(),
                vertexIndices.size(),
                remap.data());

            // 頂点データを再マッピング.
            meshopt_remapVertexBuffer(
                vertices.data(),
                dstMesh.Vertices.data(),
                dstMesh.Vertices.size(),
                sizeof(dstMesh.Vertices[0]),
                remap.data());

            meshopt_remapVertexBuffer(
                skinVertices.data(),
                dstMesh.SkinVertices.data(),
                dstMesh.SkinVertices.size(),
                sizeof(dstMesh.SkinVertices[0]),
                remap.data());


            // 不要になったメモリを解放.
            remap.clear();
            remap.shrink_to_fit();

            // 最適化したサイズにメモリ量を減らす.
            dstMesh.Vertices    .resize(vertices.size());
            dstMesh.SkinVertices.resize(skinVertices.size());
            vertexIndices       .resize(indices .size());

            // 不要になったメモリを解放.
            vertices.clear();
            vertices.shrink_to_fit();

            skinVertices.clear();
            skinVertices.shrink_to_fit();

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
        }
        else
        {
            vertexCount = meshopt_generateVertexRemap(
                remap.data(),
                vertexIndices.data(),
                vertexIndices.size(),
                dstMesh.Vertices.data(),
                dstMesh.Vertices.size(),
                sizeof(asdx::ResMeshVertex));

            std::vector<asdx::ResMeshVertex> vertices(vertexCount);
            std::vector<uint32_t> indices(vertexIndices.size());

            // 頂点インデックスを再マッピング.
            meshopt_remapIndexBuffer(
                indices.data(),
                vertexIndices.data(),
                vertexIndices.size(),
                remap.data());

            // 頂点データを再マッピング.
            meshopt_remapVertexBuffer(
                vertices.data(),
                dstMesh.Vertices.data(),
                dstMesh.Vertices.size(),
                sizeof(dstMesh.Vertices[0]),
                remap.data());

            // 不要になったメモリを解放.
            remap.clear();
            remap.shrink_to_fit();

            // 最適化したサイズにメモリ量を減らす.
            dstMesh.Vertices.resize(vertices.size());
            vertexIndices   .resize(indices .size());

            // 頂点キャッシュ最適化.
            meshopt_optimizeVertexCache(
                vertexIndices.data(),
                indices.data(),
                indices.size(),
                vertexCount);

            // 不要になったメモリを解放.
            indices.clear();
            indices.shrink_to_fit();

            meshopt_optimizeVertexFetch(
                dstMesh.Vertices.data(),
                vertexIndices.data(),
                vertexIndices.size(),
                vertices.data(),
                vertices.size(),
                sizeof(asdx::ResMeshVertex));

            // 不要になったメモリを解放.
            vertices.clear();
            vertices.shrink_to_fit();
        }
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
                dstMesh.Vertices.size(),
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
                &dstMesh.Vertices[0].Position.x,
                dstMesh.Vertices.size(),
                sizeof(dstMesh.Vertices[0]));

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
