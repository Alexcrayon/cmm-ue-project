// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameAI/Grid/GAGridMap.h"
#include "GameAI/Grid/GAGridActor.h"
#include "GACMMDataActor.generated.h"


struct FBackboneEdge {

	TArray<float> Clearances;

	TArray<FVector> PathPoints;
	//what is for?
	float Length;
};

struct FBackboneNode {
	FCellRef Cell;

	float Clearance;

	FVector WorldPosition;

	TArray<FCellRef> NeighborNodes;

	TArray<float> NeighborCosts;

	TArray<FBackboneEdge> NeighborEdge;


};

struct FBackboneGraph {
	TArray<FBackboneNode> Nodes;

	FBackboneNode* FindNode(const FCellRef& Cell)
	{
		for (FBackboneNode& Node : Nodes)
		{
			if (Node.Cell == Cell) return &Node;
		}
		return nullptr;
	}
};


UCLASS(BlueprintType, Blueprintable)
class GAMEAI_API AGACMMDataActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AGACMMDataActor();
	FGAGridMap DistanceTransform;
	FGAGridMap SkeletonMap;
	TArray<FCellRef> SkeletonCells;


	UFUNCTION(BlueprintCallable)
	void BuildDistanceTransform();

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	mutable TSoftObjectPtr<AGAGridActor> GridActor;


	UFUNCTION(BlueprintCallable)
	const AGAGridActor* GetGridActor() const;

	bool IsEdgeCell(const AGAGridActor* Grid, const FCellRef& Cell ) const;

	bool IsSkeletonCell(const AGAGridActor* Grid, const FCellRef& Cell) const;

	UFUNCTION(BlueprintCallable)
	void BuildSkeleton();

	struct FCellDist
	{
		FCellRef Cell;
		float Distance;
	};
	TArray<FCellRef> GetNeighbors(const FCellRef& Cell, const AGAGridActor* Grid) const;

	UFUNCTION(BlueprintCallable)
	void DebugDrawDistanceTransform();
	UFUNCTION(BlueprintCallable)
	void DebugDrawSkeleton();

	//colored grassfire
	UPROPERTY(BlueprintReadOnly)
	FGAGridMap ColorMap;

	UFUNCTION(BlueprintCallable)
	void BuildColoredSkeleton();

	TArray<TArray<FCellRef>> TraceAllBorderOutlines(const AGAGridActor* Grid);
	TArray<FCellRef> TraceSingleOutline(const AGAGridActor* Grid, const FCellRef& Start, TSet<FCellRef>& GlobalVisited);
	FCellRef FindNextBorderCell(const AGAGridActor* Grid, const FCellRef& Current, int32& InOutDirection, const TSet<FCellRef>& Visited) const;
	void SegmentOutline(const TArray<FCellRef>& Outline, int32 Start, int32 End, float Threshold, TArray<int32>& SplitIndices);
	void ColoredGrassfireBFS(const AGAGridActor* Grid, const TArray<FCellRef>& AllEdgeCells, const FGAGridMap& EdgeColorMap, TSet<uint64> IgnorePairs);

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bDebugColorMap = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bDebugBackbone = true;

	UFUNCTION(BlueprintCallable)
	void DebugDrawColorMap();

	UFUNCTION(BlueprintCallable)
	void BuildCMMData();

	UFUNCTION(BlueprintCallable)
	void DebugDrawBackboneGraph();

	void KeepLargestComponent();

	//void MergeSmallLoops();
	FBackboneGraph Graph;
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	const TArray<FCellRef> Offsets = {
		FCellRef(0,  1),
		FCellRef(0, -1),
		FCellRef(1,  0),
		FCellRef(-1,  0),
		FCellRef(1,  1),
		FCellRef(-1,  1),
		FCellRef(1, -1),
		FCellRef(-1, -1)
	};

	

	void BuildBackboneGraph();
	
	
	//void DeleteSkeletonBranches(int32 MinBranchLength);
	//void MergeSmallLoops();

};
