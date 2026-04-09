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
void AGACMMDataActor::BuildCMMData()
{
	AGAGridActor* Grid = GridActor.Get();
	if (!Grid) return;

	BuildDistanceTransform();
	
	// Debug step: just trace outlines and visualize them
	//TArray<TArray<FCellRef>> Outlines = TraceAllBorderOutlines(Grid);
	BuildColoredSkeleton();
	//BuildSkeleton();
	

	BuildBackboneGraph();
	
	KeepLargestComponent();
	/*if (bDebugColorMap)
	{
		DebugDrawColorMap();
	}*/

	if (bDebugBackbone) {
		DebugDrawBackboneGraph();
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

		//local max ridge check larger or equal to both, or strictly larger than one
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


//colored grassfire related:
TArray<TArray<FCellRef>> AGACMMDataActor::TraceAllBorderOutlines(const AGAGridActor* Grid)
{
	TArray<TArray<FCellRef>> AllOutlines;
	TSet<FCellRef> GlobalVisited;

	for (int32 Y = 0; Y < Grid->YCount; Y++)
	{
		for (int32 X = 0; X < Grid->XCount; X++)
		{
			FCellRef Cell(X, Y);

			if (GlobalVisited.Contains(Cell)) continue;
			if (!EnumHasAllFlags(Grid->GetCellData(Cell), ECellData::CellDataTraversable)) continue;
			if (!IsEdgeCell(Grid, Cell)) continue;

			TArray<FCellRef> Outline = TraceSingleOutline(Grid, Cell, GlobalVisited);
			if (Outline.Num() > 2)
			{
				AllOutlines.Add(Outline);
			}
		}
	}

	UE_LOG(LogTemp, Log, TEXT("TraceAllBorderOutlines - Found %d outlines."), AllOutlines.Num());
	return AllOutlines;
}


FCellRef AGACMMDataActor::FindNextBorderCell(const AGAGridActor* Grid, const FCellRef& Current, int32& InOutDirection, const TSet<FCellRef>& Visited) const
{
	// 8 directions in clockwise order
	const FCellRef Dirs[8] = {
		FCellRef(1, 0),   // right
		FCellRef(1, 1),   // bot-right
		FCellRef(0, 1),   // bot
		FCellRef(-1, 1),  // bot-left
		FCellRef(-1, 0),  // left
		FCellRef(-1, -1), // top-left
		FCellRef(0, -1),  // top
		FCellRef(1, -1)   // top-right
	};

	// Start searching from opposite direction + 1 for clockwise traversal
	int32 StartDir = (InOutDirection + 5) % 8;

	for (int32 i = 0; i < 8; i++)
	{
		int32 Dir = (StartDir + i) % 8;
		FCellRef Next(Current.X + Dirs[Dir].X, Current.Y + Dirs[Dir].Y);

		if (!Grid->IsCellRefInBounds(Next)) continue;
		if (!EnumHasAllFlags(Grid->GetCellData(Next), ECellData::CellDataTraversable)) continue;
		if (!IsEdgeCell(Grid, Next)) continue;
		if (Visited.Contains(Next)) continue;

		InOutDirection = Dir;
		return Next;
	}

	return FCellRef::Invalid;
}


TArray<FCellRef> AGACMMDataActor::TraceSingleOutline(const AGAGridActor* Grid, const FCellRef& Start, TSet<FCellRef>& GlobalVisited)
{
	TArray<FCellRef> Outline;
	FCellRef Current = Start;
	int32 Direction = 0;

	int32 MaxSteps = Grid->XCount * Grid->YCount;
	int32 Steps = 0;

	do
	{
		Outline.Add(Current);
		GlobalVisited.Add(Current);

		FCellRef Next = FindNextBorderCell(Grid, Current, Direction, GlobalVisited);

		if (!Next.IsValid())
		{
			break;
		}

		Current = Next;
		Steps++;

	} while (!(Current == Start) && Steps < MaxSteps);

	UE_LOG(LogTemp, Log, TEXT("TraceSingleOutline - Traced %d cells."), Outline.Num());
	return Outline;
}


// Ramer-Douglas-Peucker
void AGACMMDataActor::SegmentOutline(const TArray<FCellRef>& Outline, int32 Start, int32 End, float Threshold, TArray<int32>& SplitIndices)
{
	if (End - Start < 2) return;

	FVector2D LineStart(Outline[Start].X, Outline[Start].Y);
	FVector2D LineEnd(Outline[End].X, Outline[End].Y);
	FVector2D LineDir = LineEnd - LineStart;
	float LineLength = LineDir.Size();

	if (LineLength < KINDA_SMALL_NUMBER) 
		return;
	
	LineDir /= LineLength;

	float MaxDist = 0.0f;
	int32 MaxIndex = Start;

	for (int32 i = Start + 1; i < End; i++)
	{
		FVector2D Point(Outline[i].X, Outline[i].Y);
		FVector2D ToPoint = Point - LineStart;
		float Dist = FMath::Abs(ToPoint.X * LineDir.Y - ToPoint.Y * LineDir.X);

		if (Dist > MaxDist)
		{
			MaxDist = Dist;
			MaxIndex = i;
		}
	}

	if (MaxDist > Threshold)
	{
		SegmentOutline(Outline, Start, MaxIndex, Threshold, SplitIndices);
		SplitIndices.Add(MaxIndex);
		SegmentOutline(Outline, MaxIndex, End, Threshold, SplitIndices);
	}
}


// Colored BFS
void AGACMMDataActor::ColoredGrassfireBFS(const AGAGridActor* Grid, const TArray<FCellRef>& AllEdgeCells, const FGAGridMap& EdgeColorMap, TSet<uint64> IgnorePairs)
{
	// Rebuild distance transform and color map from scratch
	DistanceTransform = FGAGridMap(Grid, 0.0f);
	ColorMap = FGAGridMap(Grid, -1.0f);  // -1 = unvisited

	SkeletonCells.Empty();
	SkeletonMap = FGAGridMap(Grid, 0.0f);

	TQueue<FCellDist> Queue;
	TSet<FCellRef> SkeletonSet;

	// Enqueue all edge cells with their color
	for (const FCellRef& Cell : AllEdgeCells)
	{
		float Color;
		EdgeColorMap.GetValue(Cell, Color);
		ColorMap.SetValue(Cell, Color);
		DistanceTransform.SetValue(Cell, 0.0f);
		Queue.Enqueue({ Cell, 0.0f });
	}

	while (!Queue.IsEmpty())
	{
		FCellDist Current;
		Queue.Dequeue(Current);

		float CurrentColor;
		ColorMap.GetValue(Current.Cell, CurrentColor);

		for (const FCellRef& Offset : Offsets)
		{
			FCellRef Neighbor(Current.Cell.X + Offset.X, Current.Cell.Y + Offset.Y);

			if (!Grid->IsCellRefInBounds(Neighbor)) continue;
			if (!EnumHasAllFlags(Grid->GetCellData(Neighbor), ECellData::CellDataTraversable)) continue;

			float NeighborColor;
			ColorMap.GetValue(Neighbor, NeighborColor);

			float NewDist = Current.Distance + 1.0f;

			if (NeighborColor < 0.0f)
			{
				// Unvisited 
				ColorMap.SetValue(Neighbor, CurrentColor);
				DistanceTransform.SetValue(Neighbor, NewDist);
				Queue.Enqueue({ Neighbor, NewDist });
			}
			//  collision with another edge
			else if (NeighborColor != CurrentColor)
			{
				
				//not handle convex corner right now
				if (CurrentColor < NeighborColor)
				{
				 
					if (!SkeletonSet.Contains(Current.Cell))
					{
						SkeletonSet.Add(Current.Cell);
						SkeletonCells.Add(Current.Cell);
						float Dist;
						DistanceTransform.GetValue(Current.Cell, Dist);
						//SkeletonMap.SetValue(Current.Cell, Dist);
						SkeletonMap.SetValue(Current.Cell, 1.0f);
					}
			
				}
			
			}
		}
	}

	UE_LOG(LogTemp, Log, TEXT("ColoredGrassfireBFS - Found %d skeleton cells."), SkeletonCells.Num());
}


// Main builder
void AGACMMDataActor::BuildColoredSkeleton()
{
	AGAGridActor* Grid = GridActor.Get();
	if (!Grid) return;

	// Trace border outlines
	TArray<TArray<FCellRef>> Outlines = TraceAllBorderOutlines(Grid);

	// run rdp and assign colors
	FGAGridMap EdgeColorMap(Grid, -1.0f);
	TArray<FCellRef> AllEdgeCells;
	int32 ColorCounter = 0;
	float RDPThreshold = 1.5f;

	for (const TArray<FCellRef>& Outline : Outlines)
	{
		TArray<int32> SplitIndices;
		SplitIndices.Add(0);
		SegmentOutline(Outline, 0, Outline.Num() - 1, RDPThreshold, SplitIndices);
		SplitIndices.Add(Outline.Num() - 1);
		SplitIndices.Sort();

		for (int32 i = 0; i < SplitIndices.Num() - 1; i++)
		{
			float Color = (float)ColorCounter;

			for (int32 j = SplitIndices[i]; j <= SplitIndices[i + 1]; j++)
			{
				EdgeColorMap.SetValue(Outline[j], Color);
				AllEdgeCells.AddUnique(Outline[j]);
			}

			ColorCounter++;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("BuildColoredSkeleton - %d edge segments, %d edge cells."), ColorCounter, AllEdgeCells.Num());

	//not used 
	TSet <uint64> IgnorePairs;

	// Run colored BFS
	ColoredGrassfireBFS(Grid, AllEdgeCells, EdgeColorMap, IgnorePairs);
}



void AGACMMDataActor::BuildBackboneGraph()
{
	AGAGridActor* Grid = GridActor.Get();
	if (!Grid) return;

	Graph.Nodes.Empty();

	// Step 1: Classify skeleton cells — create nodes for dead ends and junctions
	TSet<FCellRef> NodeCells;

	for (const FCellRef& Cell : SkeletonCells)
	{
		int32 NeighborCount = 0;
		for (const FCellRef& Offset : Offsets)
		{
			FCellRef N(Cell.X + Offset.X, Cell.Y + Offset.Y);
			float Val;
			if (SkeletonMap.GetValue(N, Val) && Val > 0.0f)
			{
				NeighborCount++;
			}
		}

		// Dead end (1) or junction (3+) → becomes a node
		if (NeighborCount != 2)
		{
			FBackboneNode Node;
			Node.Cell = Cell;
			Node.WorldPosition = Grid->GetCellPosition(Cell);
			DistanceTransform.GetValue(Cell, Node.Clearance);

			NodeCells.Add(Cell);
			Graph.Nodes.Add(Node);
		}
	}

	UE_LOG(LogTemp, Log, TEXT("BuildBackboneGraph - %d nodes found"), Graph.Nodes.Num());

	// Step 2: Walk chains from each node to build adjacency lists
	// Track processed edges by storing the first chain cell to distinguish parallel paths
	TSet<FCellRef> VisitedChainStarts;

	for (FBackboneNode& StartNode :Graph.Nodes)
	{
		// Check each skeleton neighbor of this node
		for (const FCellRef& Offset : Offsets)
		{
			FCellRef NextCell(StartNode.Cell.X + Offset.X, StartNode.Cell.Y + Offset.Y);
			float Val;
			if (!SkeletonMap.GetValue(NextCell, Val) || Val <= 0.0f) 
				continue;

			// Case 1: Neighbor is another node (direct connection)
			if (NodeCells.Contains(NextCell))
			{
				// Only add from one direction to avoid duplicates
				// Use cell comparison as tiebreaker
				if (StartNode.Cell.X > NextCell.X ||
					(StartNode.Cell.X == NextCell.X && StartNode.Cell.Y > NextCell.Y)) continue;

				FBackboneEdge EdgeData;
				EdgeData.PathPoints.Add(StartNode.WorldPosition);
				FBackboneNode* EndNode = Graph.FindNode(NextCell);
				EdgeData.PathPoints.Add(EndNode->WorldPosition);
				EdgeData.Clearances.Add(StartNode.Clearance);
				EdgeData.Clearances.Add(EndNode->Clearance);
				EdgeData.Length = FVector::Dist(StartNode.WorldPosition, EndNode->WorldPosition);

				// Add to start node
				StartNode.NeighborNodes.Add(NextCell);
				StartNode.NeighborCosts.Add(EdgeData.Length);
				StartNode.NeighborEdge.Add(EdgeData);

				// Reverse for end node
				FBackboneEdge ReverseData;
				ReverseData.PathPoints = EdgeData.PathPoints;
				Algo::Reverse(ReverseData.PathPoints);
				ReverseData.Clearances = EdgeData.Clearances;
				Algo::Reverse(ReverseData.Clearances);
				ReverseData.Length = EdgeData.Length;

				EndNode->NeighborNodes.Add(StartNode.Cell);
				EndNode->NeighborCosts.Add(EdgeData.Length);
				EndNode->NeighborEdge.Add(ReverseData);

				continue;
			}

			// Case 2: Neighbor is a chain cell — walk along it
			// Use the chain start cell to track whether we've processed this edge
			if (VisitedChainStarts.Contains(NextCell)) continue;
			VisitedChainStarts.Add(NextCell);

			FBackboneEdge EdgeData;
			EdgeData.Length = 0.0f;
			EdgeData.PathPoints.Add(StartNode.WorldPosition);
			EdgeData.Clearances.Add(StartNode.Clearance);

			FCellRef CurrentCell = NextCell;
			FCellRef PrevCell = StartNode.Cell;

			while (true)
			{
				FVector CurrPos = Grid->GetCellPosition(CurrentCell);
				float CurrClearance;
				DistanceTransform.GetValue(CurrentCell, CurrClearance);

				EdgeData.Length += FVector::Dist(EdgeData.PathPoints.Last(), CurrPos);
				EdgeData.PathPoints.Add(CurrPos);
				EdgeData.Clearances.Add(CurrClearance);

				// Reached another node?
				if (NodeCells.Contains(CurrentCell))
				{
					// Mark the last chain cell before this node so the reverse walk skips it
					VisitedChainStarts.Add(PrevCell);

					FBackboneNode* EndNode = Graph.FindNode(CurrentCell);

					// Add to start node
					StartNode.NeighborNodes.Add(CurrentCell);
					StartNode.NeighborCosts.Add(EdgeData.Length);
					StartNode.NeighborEdge.Add(EdgeData);

					// Reverse for end node
					FBackboneEdge ReverseData;
					ReverseData.PathPoints = EdgeData.PathPoints;
					Algo::Reverse(ReverseData.PathPoints);
					ReverseData.Clearances = EdgeData.Clearances;
					Algo::Reverse(ReverseData.Clearances);
					ReverseData.Length = EdgeData.Length;

					EndNode->NeighborNodes.Add(StartNode.Cell);
					EndNode->NeighborCosts.Add(EdgeData.Length);
					EndNode->NeighborEdge.Add(ReverseData);

					break;
				}

				// Find next chain cell (not where we came from)
				bool bFoundNext = false;
				for (const FCellRef& ChainOffset : Offsets)
				{
					FCellRef N(CurrentCell.X + ChainOffset.X, CurrentCell.Y + ChainOffset.Y);
					float NVal;
					if (!SkeletonMap.GetValue(N, NVal) || NVal <= 0.0f) continue;
					if (N == PrevCell) continue;

					PrevCell = CurrentCell;
					CurrentCell = N;
					bFoundNext = true;
					break;
				}

				if (!bFoundNext)
				{
					UE_LOG(LogTemp, Warning, TEXT("BuildBackboneGraph - Orphaned chain at (%d, %d)"),
						CurrentCell.X, CurrentCell.Y);
					break;
				}
			}
		}
	}

	// Log summary
	int32 TotalEdges = 0;
	for (const FBackboneNode& Node : Graph.Nodes)
	{
		TotalEdges += Node.NeighborNodes.Num();
	}
	TotalEdges /= 2;

	UE_LOG(LogTemp, Log, TEXT("BuildBackboneGraph - %d nodes, %d edges"),
		Graph.Nodes.Num(), TotalEdges);
}
void AGACMMDataActor::KeepLargestComponent()
{
	// BFS from each unvisited node to find components
	TSet<FCellRef> GlobalVisited;
	TArray<TArray<FCellRef>> Components;

	for (FBackboneNode& Node : Graph.Nodes)
	{
		if (GlobalVisited.Contains(Node.Cell)) continue;

		TArray<FCellRef> Component;
		TQueue<FCellRef> Queue;
		Queue.Enqueue(Node.Cell);

		while (!Queue.IsEmpty())
		{
			FCellRef Current;
			Queue.Dequeue(Current);
			if (GlobalVisited.Contains(Current)) continue;
			GlobalVisited.Add(Current);
			Component.Add(Current);

			FBackboneNode* CurrNode = Graph.FindNode(Current);
			if (CurrNode)
			{
				for (const FCellRef& Neighbor : CurrNode->NeighborNodes)
				{
					if (!GlobalVisited.Contains(Neighbor))
					{
						Queue.Enqueue(Neighbor);
					}
				}
			}
		}

		Components.Add(Component);
	}

	// Find largest component
	int32 LargestIdx = 0;
	for (int32 i = 1; i < Components.Num(); i++)
	{
		if (Components[i].Num() > Components[LargestIdx].Num())
		{
			LargestIdx = i;
		}
	}

	// Remove nodes not in the largest component
	TSet<FCellRef> KeepSet;
	for (const FCellRef& Cell : Components[LargestIdx])
	{
		KeepSet.Add(Cell);
	}

	for (int32 i = Graph.Nodes.Num() - 1; i >= 0; i--)
	{
		if (!KeepSet.Contains(Graph.Nodes[i].Cell))
		{
			Graph.Nodes.RemoveAt(i);
		}
	}

	// Clean up neighbor references to removed nodes
	for (FBackboneNode& Node : Graph.Nodes)
	{
		for (int32 i = Node.NeighborNodes.Num() - 1; i >= 0; i--)
		{
			if (!KeepSet.Contains(Node.NeighborNodes[i]))
			{
				Node.NeighborNodes.RemoveAt(i);
				Node.NeighborCosts.RemoveAt(i);
				Node.NeighborEdge.RemoveAt(i);
			}
		}
	}


}

void AGACMMDataActor::DebugDrawColorMap()
{
	AGAGridActor* Grid = GridActor.Get();
	if (!Grid) return;

	Grid->DebugGridMap = ColorMap;
	Grid->RefreshDebugMesh();
	Grid->RefreshDebugTexture();
	Grid->DebugMeshComponent->SetVisibility(true);
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

	Grid->DebugGridMap = SkeletonMap;
	Grid->RefreshDebugMesh();
	Grid->RefreshDebugTexture();
	Grid->DebugMeshComponent->SetVisibility(true);

}


void AGACMMDataActor::DebugDrawBackboneGraph()
{
	AGAGridActor* Grid = GridActor.Get();
	if (!Grid) return;

	float ZOffset = 60.0f;

	// Draw nodes
	for (const FBackboneNode& Node : Graph.Nodes)
	{
		FVector Pos = Node.WorldPosition + FVector(0, 0, ZOffset);

		// Dead ends = yellow, junctions = red
		FColor NodeColor = (Node.NeighborNodes.Num() <= 1) ? FColor::Yellow : FColor::Red;

		//DrawDebugSphere(GetWorld(), Pos, 30.0f, 8, NodeColor, false, 30.0f);

		// Clearance circle at node
		float WorldRadius = Node.Clearance * Grid->CellScale;
		DrawDebugCircle(
			GetWorld(), Pos, WorldRadius, 32,
			FColor::Green, false, 30.0f, 0, 2.0f,
			FVector(1, 0, 0), FVector(0, 1, 0)
		);
	}

	// Draw edges — only draw when neighbor cell > own cell to avoid doubles
	for (const FBackboneNode& Node : Graph.Nodes)
	{
		for (int32 i = 0; i < Node.NeighborNodes.Num(); i++)
		{
			FCellRef NeighborCell = Node.NeighborNodes[i];

			// Only draw from lower cell to higher cell
			if (Node.Cell.X > NeighborCell.X ||
				(Node.Cell.X == NeighborCell.X && Node.Cell.Y > NeighborCell.Y)) continue;

			const FBackboneEdge& EdgeData = Node.NeighborEdge[i];

			// Draw path line
			for (int32 k = 1; k < EdgeData.PathPoints.Num(); k++)
			{
				FVector A = EdgeData.PathPoints[k - 1] + FVector(0, 0, ZOffset);
				FVector B = EdgeData.PathPoints[k] + FVector(0, 0, ZOffset);
				DrawDebugLine(GetWorld(), A, B, FColor::Cyan, true, -1.0f, 0, 3.0f);
			}

			// Draw clearance circles along edge
			for (int32 k = 0; k < EdgeData.PathPoints.Num(); k++)
			{
				FVector Pos = EdgeData.PathPoints[k] + FVector(0, 0, ZOffset);
				float WorldRadius = EdgeData.Clearances[k] * Grid->CellScale;

				DrawDebugCircle(
					GetWorld(), Pos, WorldRadius, 24,
					FColor(0, 255, 0, 60), false, 10.0f, 0, 1.0f,
					FVector(1, 0, 0), FVector(0, 1, 0)
				);
			}
		}
	}
}
