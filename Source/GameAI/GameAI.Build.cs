// Copyright Epic Games, Inc. All Rights Reserved.

using System.IO;
using UnrealBuildTool;

public class GameAI : ModuleRules
{
	public GameAI(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput", "ProceduralMeshComponent", "NavigationSystem" });
        PublicIncludePaths.AddRange(new string[] {
			Path.Combine(ModuleDirectory, "CMM")
		});
    }
}
