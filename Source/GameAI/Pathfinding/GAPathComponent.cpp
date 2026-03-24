#include "GAPathComponent.h"
#include "GameFramework/NavMovementComponent.h"
#include "Kismet/GameplayStatics.h"

UGAPathComponent::UGAPathComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	State = GAPS_None;
	bDestinationValid = false;
	ArrivalDistance = 100.0f;

	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;
}


const AGAGridActor* UGAPathComponent::GetGridActor() const
{
	if (GridActor.Get())
	{
		return GridActor.Get();
	}
	else
	{
		AGAGridActor* Result = NULL;
		AActor *GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
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

APawn* UGAPathComponent::GetOwnerPawn()
{
	AActor* Owner = GetOwner();
	if (Owner)
	{
		APawn* Pawn = Cast<APawn>(Owner);
		if (Pawn)
		{
			return Pawn;
		}
		else
		{
			AController* Controller = Cast<AController>(Owner);
			if (Controller)
			{
				return Controller->GetPawn();
			}
		}
	}

	return NULL;
}


void UGAPathComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	if (GetOwnerPawn() == NULL)
	{
		return;
	}

	bool Valid = false;
	if (bDestinationValid)
	{
		RefreshPath();
		Valid = true;
	}
	else if (bDistanceMapPathValid)
	{
		Valid = true;
	}
	if (Valid)
	{
		if (State == GAPS_Active)
		{
			FollowPath();
		}
	}

	// Super important! Otherwise, unbelievably, the Tick event in Blueprint won't get called

	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

EGAPathState UGAPathComponent::RefreshPath()
{
	AActor* Owner = GetOwnerPawn();
	if (Owner == NULL)
	{
		State = GAPS_Invalid;
		return State;
	}

	FVector StartPoint = Owner->GetActorLocation();

	check(bDestinationValid);

	float DistanceToDestination = FVector::Dist(StartPoint, Destination);

	if (DistanceToDestination <= ArrivalDistance)
	{
		// Yay! We got there!
		State = GAPS_Finished;
	}
	else
	{
		TArray<FPathStep> UnsmoothedSteps;
		Steps.Empty();

		// Replan the path!
		State = AStar(StartPoint, UnsmoothedSteps);

		// To debug A* without smoothing uncomment this line and then skip the call to SmoothPath below:
		//Steps = UnsmoothedSteps;

		if (State == EGAPathState::GAPS_Active)
		{
			// Smooth the path!
			State = SmoothPath( UnsmoothedSteps, Steps);
		}
	}

	return State;
}

EGAPathState UGAPathComponent::AStar(const FVector& StartPoint, TArray<FPathStep>& StepsOut) const
{
	const AGAGridActor* Grid = GetGridActor();

	// Assignment 2 Part3: replace this with an A* search!
	// HINT 1: you made need a heap structure. A TArray can be accessed as a heap -- just add/remove elements using
	// the TArray::HeapPush() and TArray::HeapPop() methods.
	// Note that whatever you push or pop needs to implement the 'less than' operator (operator<)
	// HINT 2: UE has some useful flag testing function. For example you can test for traversability by doing this:
	// ECellData Flags = Grid->GetCellData(CellRef);
	// bool bIsCellTraversable = EnumHasAllFlags(Flags, ECellData::CellDataTraversable)
	FCellRef startCell = Grid->GetCellRef(StartPoint);

	TArray<CellCost> pq;

	pq.HeapPush(CellCost(startCell, 0.0f));
	
	TMap<FCellRef, float> gCost;
	TArray<FCellRef> visited;
	TMap<FCellRef, FCellRef> CameFrom;

	//loop over each cell on the grid to initialize
	for (double i = 0; i < Grid->XCount; i++) {
		for (double j = 0; j < Grid->YCount; j++) {
			FCellRef cell(i, j);
			gCost.Add(cell, TNumericLimits<float>::Max());
		}
	}
	gCost.Add(startCell, 0.0f);
	
	while (pq.Num() > 0) {
		
		CellCost curr;
		pq.HeapPop(curr);
		FCellRef currCell = curr.Cell;
		if (!visited.Contains(currCell)) {
			visited.Add(currCell);

			// hit goal
			// start backtracing path
			if (currCell == DestinationCell) {
				TArray<FCellRef> Path = TracePath(CameFrom, startCell,DestinationCell);
				
				StepsOut.SetNum(Path.Num());
				for (int i = 0; i < Path.Num(); i++) {
					StepsOut[i].Set(Grid->GetCellPosition(Path[i]), Path[i]);
				}

				return GAPS_Active;
			}
			// check each neighbor
			for (FCellRef n : GetNeighbors(currCell, Grid)) {
				float g = *gCost.Find(currCell) + currCell.Distance(n);
				if (g < *gCost.Find(n)) {

					gCost.Add(n, g);
					float h = Heuristic(n, DestinationCell);

					CameFrom.Add(n,currCell);
					
					pq.HeapPush(CellCost(n, g + h));
				}
			}
		}
	}
	return GAPS_Invalid;
}
TArray<FCellRef> UGAPathComponent::TracePath(const TMap<FCellRef, FCellRef>& CameFrom, FCellRef Start, FCellRef Dest) const{
	TArray<FCellRef> Path;

	FCellRef BackTrace = Dest;
	while (BackTrace!=Start) {
		Path.Add(BackTrace);

		const FCellRef* Prev;

		Prev = CameFrom.Find(BackTrace);

		BackTrace = *Prev;
	}
	Algo::Reverse(Path);
	return Path;
}

TArray<FCellRef> UGAPathComponent::GetNeighbors(const FCellRef& Cell, const AGAGridActor* Grid) const{
	
	TArray<FCellRef> Neighbors;
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
float UGAPathComponent::Heuristic(const FCellRef& Cell, const FCellRef& Dest) const {
	return Cell.Distance(Dest);

}

bool UGAPathComponent::Dijkstra(const FVector& StartPoint, FGAGridMap& DistanceMapOut,TMap<FCellRef, FCellRef>& PrevNodeOut) const
{
	// Assignment 3 Part 3-1: implement Dijkstra's algorithm to fill out the distance map

	const AGAGridActor* Grid = GetGridActor();
	FCellRef startCell = Grid->GetCellRef(StartPoint);

	TArray<CellCost> pq;

	pq.HeapPush(CellCost(startCell, 0.0f));

	TMap<FCellRef, float> Cost;
	TArray<FCellRef> visited;

	//loop over each cell on the grid to initialize
	for (double i = 0; i < Grid->XCount; i++) {
		for (double j = 0; j < Grid->YCount; j++) {
			FCellRef cell(i, j);
			Cost.Add(cell, TNumericLimits<float>::Max());
		}
	}
	Cost.Add(startCell, 0.0f);

	while (pq.Num() > 0) {

		CellCost curr;
		pq.HeapPop(curr);
		FCellRef currCell = curr.Cell;
		if (!visited.Contains(currCell)) {
			visited.Add(currCell);
			// check each neighbor
			for (FCellRef n : GetNeighbors(currCell, Grid)) {
				float g = *Cost.Find(currCell) + currCell.Distance(n);
					
				if (g < *Cost.Find(n)) {

					Cost.Add(n, g);
					//float h = Heuristic(n, DestinationCell);

					DistanceMapOut.SetValue(n, g);
					PrevNodeOut.Add(n, currCell);

					pq.HeapPush(CellCost(n, g));
				}
			}
		}
	}
	
	
	return visited.Num() > 0;
}

bool UGAPathComponent::BuildPathFromDistanceMap(const FVector& EndPoint, const FCellRef& EndCellRef, const FGAGridMap& DistanceMap, const TMap<FCellRef, FCellRef>& PrevNodeMap)
{
	bDistanceMapPathValid = false;

	// Assignment 3 Part 3-2: reconstruct a path from the distance map

	// Remember to smooth the path as well, using your existing smoothing code

	// Set this to true when you've successfully built the path
	// bDistanceMapPathValid = true;

	const AGAGridActor* Grid = GetGridActor();

	
	//need to know whether path was built successfully
	//START
	AActor* Owner = GetOwnerPawn();
	FVector StartPoint = Owner->GetActorLocation();
	FCellRef StartCell = Grid->GetCellRef(StartPoint);
	
	TArray<FCellRef> Path = TracePath(PrevNodeMap, StartCell, EndCellRef);
	

	
	
	Steps.SetNum(Path.Num());
	for (int i = 0; i < Path.Num(); i++) {
		Steps[i].Set(Grid->GetCellPosition(Path[i]), Path[i]);

	}

	TArray<FPathStep> SmoothedStep;
	SmoothPath(Steps, SmoothedStep);
	Steps = SmoothedStep;

	if (Steps.Num() > 0) {
		bDistanceMapPathValid = true;
	}
	

	if (bDistanceMapPathValid)
	{
		// once you have built the path (i.e. filled in the Steps array in the GAPathComponent), set the path component's state to GAPS_Active
		// This will cause 
		State = GAPS_Active;
	}

	return bDistanceMapPathValid;
}


EGAPathState UGAPathComponent::SmoothPath( const TArray<FPathStep>& UnsmoothedSteps, TArray<FPathStep>& SmoothedStepsOut) const
{
	// Assignment 2 Part 4: smooth the path
	// High level description from the lecture:
	// * Trace to each subsequent step until you collide
	// * Back up one step (to the last clear one)
	// * Add that cell to your smoothed step
	// * Start again from there

	//SmoothedStepsOut.Add(UnsmoothedSteps[0]);

	int32 Current = 0; 
	int32 Last = UnsmoothedSteps.Num() - 1; // destination cell

	
	while (Current < Last)
	{
		int32 ReachableIndex = Current + 1;  //intialize to the next cell after start

		// Try furthest first, go backwards
		for (int32 i = Last; i > Current + 1; i--)
		{
			FCellRef From = UnsmoothedSteps[Current].CellRef;
			FCellRef To = UnsmoothedSteps[i].CellRef;

			if (LineTrace(From, To))
			{
				ReachableIndex = i;
				break;
			}
		}

		SmoothedStepsOut.Add(UnsmoothedSteps[ReachableIndex]);
		Current = ReachableIndex;
	}

	return GAPS_Active;
}

bool UGAPathComponent::LineTrace(const FCellRef& Start, const FCellRef& End) const {
	const AGAGridActor* Grid = GetGridActor();

	//choose the larger X/Y differences as steps
	int32 steps = FMath::Max(FMath::Abs(End.X - Start.X), FMath::Abs(End.Y - Start.Y));

	if (steps == 0)
	{
		return true;
	}

	for (int32 i = 0; i <= steps; i++)
	{
		float Alpha = static_cast<float>(i) / static_cast<float>(steps);

		//using linear interpolation to find a nearby cell
		int32 X = FMath::RoundToInt(FMath::Lerp(static_cast<float>(Start.X), static_cast<float>(End.X), Alpha));
		int32 Y = FMath::RoundToInt(FMath::Lerp(static_cast<float>(Start.Y), static_cast<float>(End.Y), Alpha));

		FCellRef Cell(X, Y);

		if (!Grid->IsCellRefInBounds(Cell))
		{
			return false;
		}

		ECellData Flags = Grid->GetCellData(Cell);
		if (!EnumHasAllFlags(Flags, ECellData::CellDataTraversable))
		{
			return false;
		}
	}

	return true;
}
void UGAPathComponent::FollowPath()
{
	AActor* Owner = GetOwnerPawn();
	if (Owner == NULL)
	{
		return;
	}

	FVector StartPoint = Owner->GetActorLocation();

	check(State == GAPS_Active);
	check(Steps.Num() > 0);

	// Always follow the first step, assuming that we are refreshing the whole path every tick
	FVector V = Steps[0].Point - StartPoint;
	V.Normalize();

	UNavMovementComponent* MovementComponent = Owner->FindComponentByClass<UNavMovementComponent>();
	if (MovementComponent)
	{
		MovementComponent->RequestPathMove(V);
	}
}


EGAPathState UGAPathComponent::SetDestination(const FVector &DestinationPoint)
{
	Destination = DestinationPoint;

	State = GAPS_Invalid;
	bDestinationValid = true;

	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		FCellRef CellRef = Grid->GetCellRef(Destination);
		if (CellRef.IsValid())
		{
			DestinationCell = CellRef;
			bDestinationValid = true;

			RefreshPath();
		}
	}

	return State;
}