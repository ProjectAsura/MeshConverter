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
        if (pMesh->HasBones())
        { ParseSkinningMesh(model, pMesh); }
        else
        { ParseStaticMesh(model, pMesh); }
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
void MeshLoader::ParseStaticMesh(asdx::ResModel& model, const aiMesh* pSrcMesh)
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

    asdx::ResStaticMesh dstMesh;

    aiVector3D zero3D(0.0f, 0.0f, 0.0f);
    aiColor4D  white (1.0f, 1.0f, 1.0f, 1.0f);

    // 頂点データのメモリを確保.
    dstMesh.Vertices.resize(pSrcMesh->mNumVertices);

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
    }

    // 頂点インデックスのメモリを確保.
    std::vector<uint32_t>   dstMeshIndices;
    dstMeshIndices.resize(pSrcMesh->mNumFaces * 3);

    for(auto i=0u; i<pSrcMesh->mNumFaces; ++i)
    {
        const auto& face = pSrcMesh->mFaces[i];
        assert(face.mNumIndices == 3);  // 三角形化しているので必ず3になっている.

        dstMeshIndices[i * 3 + 0] = face.mIndices[0];
        dstMeshIndices[i * 3 + 1] = face.mIndices[1];
        dstMeshIndices[i * 3 + 2] = face.mIndices[2];
    }

    // 最適化.
    {
        std::vector<uint32_t> remap(dstMesh.Indices.size());

        // 重複データを削除するための再マッピング用インデックスを生成.
        auto vertexCount = meshopt_generateVertexRemap(
            remap.data(),
            dstMeshIndices.data(),
            dstMeshIndices.size(),
            dstMesh.Vertices.data(),
            dstMesh.Vertices.size(),
            sizeof(asdx::ResMeshVertex));

        std::vector<asdx::ResMeshVertex> vertices(vertexCount);
        std::vector<uint32_t> indices(dstMesh.Indices.size());

        // 頂点インデックスを再マッピング.
        meshopt_remapIndexBuffer(
            indices.data(),
            dstMeshIndices.data(),
            dstMeshIndices.size(),
            remap.data());

        // 頂点データを再マッピング.
        meshopt_remapVertexBuffer(
            vertices.data(),
            dstMesh.Vertices.data(),
            dstMesh.Vertices.size(),
            sizeof(asdx::ResMeshVertex),
            remap.data());

        // 不要になったメモリを解放.
        remap.clear();
        remap.shrink_to_fit();

        // 最適化したサイズにメモリ量を減らす.
        dstMesh.Vertices.resize(vertices.size());
        dstMeshIndices .resize(indices .size());

        // 頂点キャッシュ最適化.
        meshopt_optimizeVertexCache(
            dstMeshIndices.data(),
            indices.data(),
            indices.size(),
            vertexCount);

        // 不要になったメモリを解放.
        indices.clear();
        indices.shrink_to_fit();

        // 頂点フェッチ最適化.
        meshopt_optimizeVertexFetch(
            dstMesh.Vertices.data(),
            dstMeshIndices.data(),
            dstMeshIndices.size(),
            vertices.data(),
            vertices.size(),
            sizeof(asdx::ResMeshVertex));

        // 不要になったメモリを解放.
        vertices.clear();
        vertices.shrink_to_fit();
    }

    // メッシュレット生成.
    {
        const size_t kMaxVertices   = 64;
        const size_t kMaxPrimitives = 124;

        std::vector<meshopt_Meshlet> meshlets(
            meshopt_buildMeshletsBound(
                dstMeshIndices.size(),
                kMaxVertices,
                kMaxPrimitives));
        meshlets.resize(
            meshopt_buildMeshlets(
                meshlets.data(),
                dstMeshIndices.data(),
                dstMeshIndices.size(),
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
                sizeof(asdx::ResMeshVertex));

            // メッシュレットデータ設定.
            asdx::ResMeshlet m = {};
            m.VertexCount       = meshlet.vertex_count;
            m.VertexOffset      = vertexOffset;
            m.PrimitiveCount    = meshlet.triangle_count;
            m.PrimitiveOffset   = primitiveOffset;

            dstMesh.Meshlets.push_back(m);
        }

        // サイズ最適化.
        dstMesh.Indices     .shrink_to_fit();
        dstMesh.Primitives  .shrink_to_fit();
        dstMesh.Meshlets    .shrink_to_fit();
    }

    model.StaticMeshes.push_back(dstMesh);
}

//-----------------------------------------------------------------------------
//      スキニングメッシュデータを解析します.
//-----------------------------------------------------------------------------
void MeshLoader::ParseSkinningMesh(asdx::ResModel& model, const aiMesh* pSrcMesh)
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

    asdx::ResSkinningMesh dstMesh;

    aiVector3D zero3D(0.0f, 0.0f, 0.0f);
    aiColor4D white(1.0f, 1.0f, 1.0f, 1.0f);

    // 頂点データのメモリを確保.
    dstMesh.Vertices.resize(pSrcMesh->mNumVertices);

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
        dstMesh.Vertices[i].BoneIndex       = asdx::ResBoneIndex(0, 0, 0, 0);
        dstMesh.Vertices[i].BoneWeights     = asdx::Vector4(0.0f, 0.0f, 0.0f, 0.0f);
    }

    for(auto i=0u; i<pSrcMesh->mNumBones; ++i)
    {
        for(auto j=0u; j<pSrcMesh->mBones[i]->mNumWeights; ++j)
        {
            auto weight     = pSrcMesh->mBones[i]->mWeights[j];
            auto boneIndex  = uint16_t(i);
            auto boneWeight = weight.mWeight;

            auto& vertex = dstMesh.Vertices[weight.mVertexId];
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
    std::vector<uint32_t>   dstMeshIndices;
    dstMeshIndices.resize(pSrcMesh->mNumFaces * 3);

    for(auto i=0u; i<pSrcMesh->mNumFaces; ++i)
    {
        const auto& face = pSrcMesh->mFaces[i];
        assert(face.mNumIndices == 3);  // 三角形化しているので必ず3になっている.

        dstMeshIndices[i * 3 + 0] = face.mIndices[0];
        dstMeshIndices[i * 3 + 1] = face.mIndices[1];
        dstMeshIndices[i * 3 + 2] = face.mIndices[2];
    }

    // 最適化.
    {
        std::vector<uint32_t> remap(dstMesh.Indices.size());

        // 重複データを削除するための再マッピング用インデックスを生成.
        auto vertexCount = meshopt_generateVertexRemap(
            remap.data(),
            dstMeshIndices.data(),
            dstMeshIndices.size(),
            dstMesh.Vertices.data(),
            dstMesh.Vertices.size(),
            sizeof(asdx::ResSkinningMeshVertex));

        std::vector<asdx::ResSkinningMeshVertex> vertices(vertexCount);
        std::vector<uint32_t> indices(dstMesh.Indices.size());

        // 頂点インデックスを再マッピング.
        meshopt_remapIndexBuffer(
            indices.data(),
            dstMeshIndices.data(),
            dstMeshIndices.size(),
            remap.data());

        // 頂点データを再マッピング.
        meshopt_remapVertexBuffer(
            vertices.data(),
            dstMesh.Vertices.data(),
            dstMesh.Vertices.size(),
            sizeof(asdx::ResSkinningMeshVertex),
            remap.data());

        // 不要になったメモリを解放.
        remap.clear();
        remap.shrink_to_fit();

        // 最適化したサイズにメモリ量を減らす.
        dstMesh.Vertices.resize(vertices.size());
        dstMeshIndices  .resize(indices .size());

        // 頂点キャッシュ最適化.
        meshopt_optimizeVertexCache(
            dstMeshIndices.data(),
            indices.data(),
            indices.size(),
            vertexCount);

        // 不要になったメモリを解放.
        indices.clear();
        indices.shrink_to_fit();

        // 頂点フェッチ最適化.
        meshopt_optimizeVertexFetch(
            dstMesh.Vertices.data(),
            dstMeshIndices.data(),
            dstMeshIndices.size(),
            vertices.data(),
            vertices.size(),
            sizeof(asdx::ResSkinningMeshVertex));

        // 不要になったメモリを解放.
        vertices.clear();
        vertices.shrink_to_fit();
    }

    // メッシュレット生成.
    {
        const size_t kMaxVertices   = 64;
        const size_t kMaxPrimitives = 124;

        std::vector<meshopt_Meshlet> meshlets(
            meshopt_buildMeshletsBound(
                dstMeshIndices.size(),
                kMaxVertices,
                kMaxPrimitives));
        meshlets.resize(
            meshopt_buildMeshlets(
                meshlets.data(),
                dstMeshIndices.data(),
                dstMeshIndices.size(),
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
                sizeof(asdx::ResSkinningMeshVertex));

            // メッシュレットデータ設定.
            asdx::ResMeshlet m = {};
            m.VertexCount       = meshlet.vertex_count;
            m.VertexOffset      = vertexOffset;
            m.PrimitiveCount    = meshlet.triangle_count;
            m.PrimitiveOffset   = primitiveOffset;

            dstMesh.Meshlets.push_back(m);
        }

        // サイズ最適化.
        dstMesh.Indices     .shrink_to_fit();
        dstMesh.Primitives  .shrink_to_fit();
        dstMesh.Meshlets    .shrink_to_fit();
    }

    model.SkinningMeshes.push_back(dstMesh);
}

////-----------------------------------------------------------------------------
////      マテリアルデータを解析します.
////-----------------------------------------------------------------------------
//void MeshLoader::ParseMaterial(ResMaterial& dstMaterial, const aiMaterial* pSrcMaterial)
//{
//    // 拡散反射成分.
//    {
//        aiColor3D color(0.0f, 0.0f, 0.0f);
//
//        if (pSrcMaterial->Get(AI_MATKEY_COLOR_DIFFUSE, color) == AI_SUCCESS)
//        {
//            dstMaterial.Diffuse.x = color.r;
//            dstMaterial.Diffuse.y = color.g;
//            dstMaterial.Diffuse.z = color.b;
//        }
//        else
//        {
//            dstMaterial.Diffuse.x = 0.5f;
//            dstMaterial.Diffuse.y = 0.5f;
//            dstMaterial.Diffuse.z = 0.5f;
//        }
//    }
//
//    // 鏡面反射成分.
//    {
//        aiColor3D color(0.0f, 0.0f, 0.0f);
//
//        if (pSrcMaterial->Get(AI_MATKEY_COLOR_SPECULAR, color) == AI_SUCCESS)
//        {
//            dstMaterial.Specular.x = color.r;
//            dstMaterial.Specular.y = color.g;
//            dstMaterial.Specular.z = color.b;
//        }
//        else
//        {
//            dstMaterial.Specular.x = 0.0f;
//            dstMaterial.Specular.y = 0.0f;
//            dstMaterial.Specular.z = 0.0f;
//        }
//    }
//
//    // 鏡面反射強度.
//    {
//        float shininess = 0.0f;
//        if (pSrcMaterial->Get(AI_MATKEY_SHININESS, shininess) == AI_SUCCESS)
//        { dstMaterial.Shininess = shininess; }
//        else
//        { dstMaterial.Shininess = 0.0f; }
//    }
//
//    // ディフューズマップ.
//    {
//        aiString path;
//        if (pSrcMaterial->Get(AI_MATKEY_TEXTURE_DIFFUSE(0), path) == AI_SUCCESS)
//        { dstMaterial.DiffuseMap = Convert(path); }
//        else
//        { dstMaterial.DiffuseMap.clear(); }
//    }
//
//    // スペキュラーマップ.
//    {
//        aiString path;
//        if (pSrcMaterial->Get(AI_MATKEY_TEXTURE_SPECULAR(0), path) == AI_SUCCESS)
//        { dstMaterial.SpecularMap = Convert(path); }
//        else
//        { dstMaterial.SpecularMap.clear(); }
//    }
//
//    // シャイネスマップ.
//    {
//        aiString path;
//        if (pSrcMaterial->Get(AI_MATKEY_TEXTURE_SHININESS(0), path) == AI_SUCCESS)
//        { dstMaterial.ShininessMap = Convert(path); }
//        else
//        { dstMaterial.ShininessMap.clear(); }
//    }
//
//    // 法線マップ
//    {
//        aiString path;
//        if (pSrcMaterial->Get(AI_MATKEY_TEXTURE_NORMALS(0), path) == AI_SUCCESS)
//        { dstMaterial.NormalMap = Convert(path); }
//        else
//        {
//            if (pSrcMaterial->Get(AI_MATKEY_TEXTURE_HEIGHT(0), path) == AI_SUCCESS)
//            { dstMaterial.NormalMap = Convert(path); }
//            else
//            { dstMaterial.NormalMap.clear(); }
//        }
//    }
//}

//} // namespace
