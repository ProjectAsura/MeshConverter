//-----------------------------------------------------------------------------
// File : main.cpp
// Desc : Main Entry Point.
// Copyright(c) Project Asura. All right reserved.
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <MeshLoader.h>
#include <asdxLogger.h>


//-----------------------------------------------------------------------------
//      テクスチャ用途を文字列にします.
//-----------------------------------------------------------------------------
const char* ToString(TEXTURE_USAGE usage)
{
    const char* table[] = {
        "NONE",
        "DIFFUSE",
        "SPECULAR",
        "AMBIENT",
        "EMISSIVE",
        "HEIGHT",
        "NORMAL",
        "SHININESS",
        "OPACITY",
        "DISPLACEMENT",
        "LIGHTMAP",
        "REFLECTION"
    };

    return table[usage];
}

//-----------------------------------------------------------------------------
//      マテリアル情報をYAMLファイルに出力します.
//-----------------------------------------------------------------------------
bool ExportMaterialYaml(const char* name, const std::vector<Material>& materials)
{
    FILE* pFile;
    auto err = fopen_s(&pFile, name, "w");
    if (err != 0)
    {
        ELOGA("Error : File Open Failed. path = %s", name);
        return false;
    }

    fprintf_s(pFile, "# Materials\n");

    for(size_t i=0; i<materials.size(); ++i)
    {
        auto& mat = materials[i];
        fprintf_s(pFile, "- name: %s\n", mat.Name.c_str());
        fprintf_s(pFile, "  hash: %u\n", mat.Hash);

        if (mat.Textures.size() > 0)
        {
            fprintf_s(pFile, "  textures:\n");
            for(size_t j=0; j<mat.Textures.size(); ++j)
            {
                auto& tex = mat.Textures[j];
                fprintf_s(pFile, "    - usage: %s\n", ToString(tex.Usage));
                fprintf_s(pFile, "      path: %s\n", tex.Path.c_str());
            }
        }
        fprintf_s(pFile, "\n");
    }

    fclose(pFile);
    return true;
}

//-----------------------------------------------------------------------------
//      メインエントリーポイントです.
//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    if (argc <= 1)
    {
        return 0;
    }

    std::string input;
    std::string output;
    std::string matyaml;

    for(auto i=0; i<argc; ++i)
    {
        if (strcmp(argv[i], "-i") == 0)
        {
            i++;
            input = argv[i];
        }
        else if (strcmp(argv[i], "-o") == 0)
        {
            i++;
            output = argv[i];
        }
        else if (strcmp(argv[i], "-m") == 0)
        {
            i++;
            matyaml = argv[i];
        }
    }

    asdx::ResModel model;
    MeshLoader loader;
    if (!loader.Load(input.c_str(), model))
    {
        ELOGA("Error : MeshLoader::Load() Failed. path = %s", input.c_str());
        return -1;
    }

    if (!matyaml.empty())
    {
       if (ExportMaterialYaml(matyaml.c_str(), loader.GetMaterials()))
       { ILOGA("Info : Material Save OK! output path = %s", matyaml.c_str()); }
       else
       {
           ELOGA("Error : ExportMaterialYaml() Failed. path = %s", matyaml.c_str());
           return -1;
       }
    }

    if (!asdx::SaveModel(output.c_str(), model))
    {
        ELOGA("Error : SaveModel() Fialed. path = %s", output.c_str());
        return -1;
    }

    ILOGA("Info : Model Save OK! output path = %s", output.c_str());

    return 0;
}