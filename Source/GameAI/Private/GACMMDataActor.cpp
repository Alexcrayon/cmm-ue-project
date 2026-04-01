// Fill out your copyright notice in the Description page of Project Settings.


#include "GACMMDataActor.h"
#include "GameFramework/NavMovementComponent.h"
#include "Kismet/GameplayStatics.h"
#include "ProceduralMeshComponent.h"

// Sets default values
AGACMMDataActor::AGACMMDataActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AGACMMDataActor::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AGACMMDataActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

const AGAGridActor* AGACMMDataActor::GetGridActor() const
{
	if (GridActor.Get())
	{
		return GridActor.Get();
	}
	else
	{
		AGAGridActor* Result = NULL;
		AActor* GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
		if (GenericResult)
		{
			Result = Cast<AGAGridActor>(GenericResult);
			if (Result)
			{
				// Cache the result
				// Note, GridActor is marked as mutable in the header, which is why this is allowed in a const method
				GridActor = Result;
			}
		}

		return Result;
	}
}

void AGACMMDataActor::BuildDistanceTransform() {
	AGAGridActor* Grid = GridActor.Get();
	if (!Grid)
	{
		UE_LOG(LogTemp, Log, TEXT("AGACMMDataActor::Grid not found."));
		return;
	}

	DistanceTransform = FGAGridMap(Grid, 0.0f);
	TQueue<FCellDist> Queue;
	TSet<FCellRef> Visited;
	for (int32 X = 0; X < Grid->XCount; X++)
	{
		for (int32 Y = 0; Y < Grid->YCount; Y++)
		{
			FCellRef Cell(X, Y);
			ECellData Flags = Grid->GetCellData(Cell);

			if (EnumHasAllFlags(Flags, ECellData::CellDataTraversable))
			{
				if (IsEdgeCell(Grid, Cell))
				{
					DistanceTransform.SetValue(Cell, 0.0f);
					Queue.Enqueue({ Cell, 0.0f });
					Visited.Add(Cell);
				}
			}
		}
	}


	
	//doing bfs
	while (!Queue.IsEmpty()) {
		FCellDist Current;
		Queue.Dequeue(Current);

		for (FCellRef N : GetNeighbors(Current.Cell, Grid)) {
			float Value;
			DistanceTransform.GetValue(N, Value);
			
			
			//skip visited cells
			if (Visited.Contains(N)) {
				continue;
			}

			//check diagonal
			/*		bool IsDiagonal = FMath::Abs(Current.Cell.X - N.X) == 1 
							&& FMath::Abs(Current.Cell.Y - N.Y) == 1;
			if (FMath::Abs(Current.Cell.X - N.X) == 1 && FMath::Abs(Current.Cell.Y - N.Y) == 1) {

			}*/

			float NewDist = Current.Distance + 1.0f;
			DistanceTransform.SetValue(N, NewDist);
			
			Queue.Enqueue(FCellDist(N, NewDist));
			Visited.Add(N);
		}
		

	}


	UE_LOG(LogTemp, Log, TEXT("AGACMMDataActor::BuildDistanceTransform Complete."));

	//for debugging
	int32 EdgeCount = 0;
	int32 UnvisitedCount = 0;
	float MaxDist = 0.0f;
	for (int32 Y = 0; Y < Grid->YCount; Y++)
	{
		for (int32 X = 0; X < Grid->XCount; X++)
		{
			float Val;
			DistanceTransform.GetValue(FCellRef(X, Y), Val);
			if (Val == 0.0f) EdgeCount++;
			if (Val < 0.0f) UnvisitedCount++;
			if (Val > MaxDist) MaxDist = Val;
		}
	}

	int32 TraversableCount = 0;
	for (int32 Y = 0; Y < Grid->YCount; Y++)
	{
		for (int32 X = 0; X < Grid->XCount; X++)
		{
			if (EnumHasAllFlags(Grid->GetCellData(FCellRef(X, Y)), ECellData::CellDataTraversable))
			{
				TraversableCount++;
			}
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("Total cells: %d, Traversable: %d, Edge: %d, Unvisited: %d, MaxDist: %f"),
		Grid->XCount * Grid->YCount, TraversableCount, EdgeCount, UnvisitedCount, MaxDist);
	//UE_LOG(LogTemp, Log, TEXT("AGACMMDataActor::BuildDistanceTransform Complete."));
}

bool AGACMMDataActor::IsEdgeCell(const AGAGridActor* Grid, const FCellRef& Cell) const {
	// A traversable cell is an edge cell if any of its 8 neighbors
	// is non-traversable or out of bounds
	
	// First check: is myself even traversable
	if (!EnumHasAllFlags(Grid->GetCellData(Cell), ECellData::CellDataTraversable))
	{
		return false;
	}



	for (const FCellRef& Offset : Offsets)
	{
		FCellRef Neighbor(Cell.X + Offset.X, Cell.Y + Offset.Y);

		if (!Grid->IsCellRefInBounds(Neighbor))
		{
			// Neighbor is out of bounds — this cell is on the grid edge
			return true;
		}

		ECellData Flags = Grid->GetCellData(Neighbor);
		if (!EnumHasAllFlags(Flags, ECellData::CellDataTraversable))
		{
			// Neighbor is a wall
			return true;
		}
	}

	return false;


}


bool AGACMMDataActor::IsSkeletonCell(const AGAGridActor* Grid, const FCellRef& Cell) const {

	float CurrentVal;
	DistanceTransform.GetValue(Cell, CurrentVal);

	//ignore edge cells
	if (CurrentVal <= 0.0f)
		return false;

	// Four opposing neighbor pairs: left-right, up-down, two diagonals
	FCellRef NeighborPairs[4][2] = {
		{ FCellRef(Cell.X - 1, Cell.Y),     FCellRef(Cell.X + 1, Cell.Y) },     // left-right
		{ FCellRef(Cell.X, Cell.Y - 1),     FCellRef(Cell.X, Cell.Y + 1) },     // up-down
		{ FCellRef(Cell.X - 1, Cell.Y - 1), FCellRef(Cell.X + 1, Cell.Y + 1) }, // diagonal TopLeft to botR
		{ FCellRef(Cell.X + 1, Cell.Y - 1), FCellRef(Cell.X - 1, Cell.Y + 1) }  // diagonal TopRight to botL
	};

	for (int32 i = 0; i < 4; i++)
	{
		// For out-of-bounds or non-traversable neighbors, treat as 0 (wall)
		float ValA = 0.0f;
		float ValB = 0.0f;
		if (Grid->IsCellRefInBounds(NeighborPairs[i][0]))
		{
			DistanceTransform.GetValue(NeighborPairs[i][0], ValA);
		}

		if (Grid->IsCellRefInBounds(NeighborPairs[i][1]))
		{
			DistanceTransform.GetValue(NeighborPairs[i][1], ValB);
		}

		/*if (ValA == 0.0f || ValB == 0.0f)
		{
			continue;
		}*/
		//ridge check larger or equal to both, or strictly larger than one
		if (CurrentVal >= ValA && CurrentVal >=  ValB && (CurrentVal > ValA || CurrentVal > ValB)) {
			return true;
		}
	}

	return false;
}

void AGACMMDataActor::BuildSkeleton() {
	AGAGridActor* Grid = GridActor.Get();
	if (!Grid) return;

	SkeletonCells.Empty();
	SkeletonMap = FGAGridMap(Grid, 0.0f);

	for (int32 X = 0; X < Grid->YCount; X++)
	{
		for (int32 Y = 0; Y < Grid->XCount; Y++)
		{
			FCellRef Cell(X, Y);

			if (!EnumHasAllFlags(Grid->GetCellData(Cell), ECellData::CellDataTraversable))
			{
				continue;
			}

			if (IsSkeletonCell(Grid, Cell))
			{
				SkeletonCells.Add(Cell);
				// Store the clearance value from distance transform
				float Clearance;
				DistanceTransform.GetValue(Cell, Clearance);
				SkeletonMap.SetValue(Cell, 1.0f);
			}
		}
	}

	UE_LOG(LogTemp, Log, TEXT("AGACMMDataActor::BuildSkeleton — Found %d skeleton cells."), SkeletonCells.Num());

}


TArray<FCellRef> AGACMMDataActor::GetNeighbors(const FCellRef& Cell, const AGAGridActor* Grid) const {

	TArray<FCellRef> Neighbors;

	for (const FCellRef& offset : Offsets) {
		FCellRef neighbor(Cell.X + offset.X, Cell.Y + offset.Y);
		if (Grid->IsCellRefInBounds(neighbor)) {
			ECellData flags = Grid->GetCellData(neighbor);
			bool bIsCellTraversable = EnumHasAllFlags(flags, ECellData::CellDataTraversable);
			if (bIsCellTraversable) {
				Neighbors.Add(neighbor);
			}
		}
	}

	return Neighbors;
}


void AGACMMDataActor::DebugDrawDistanceTransform()
{
	AGAGridActor* Grid = GridActor.Get();
	if (!Grid)
	{
		UE_LOG(LogTemp, Log, TEXT("AGACMMDataActor::Grid not found."));

		return;
	}

	// Reuse the grid actor's debug texture system
	Grid->DebugGridMap = DistanceTransform;
	Grid->RefreshDebugTexture();
	Grid->DebugMeshComponent->SetVisibility(true);
}


void AGACMMDataActor::DebugDrawSkeleton()
{
	AGAGridActor* Grid = GridActor.Get();
	if (!Grid) return;

	//// Option 1: Use the debug texture
	Grid->DebugGridMap = SkeletonMap;
	Grid->RefreshDebugMesh();
	Grid->RefreshDebugTexture();
	Grid->DebugMeshComponent->SetVisibility(true);

}
