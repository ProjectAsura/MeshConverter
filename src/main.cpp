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
    }

    asdx::ResModel model;
    MeshLoader loader;
    if (!loader.Load(input.c_str(), model))
    {
        ELOG("Error : MeshLoader::Load() Failed. path = %s", input.c_str());
        return -1;
    }

    if (!asdx::SaveModel(output.c_str(), model))
    {
        ELOG("Error : SaveModel() Fialed. path = %s", output.c_str());
        return -1;
    }

    ILOG("Info : Model Save Successed!! output path = %s", output.c_str());

    return 0;
}