#include "GATargetComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GameAI/Grid/GAGridActor.h"
#include "GAPerceptionSystem.h"
#include "ProceduralMeshComponent.h"



UGATargetComponent::UGATargetComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;

	SetTickGroup(ETickingGroup::TG_PostUpdateWork);

	// Generate a new guid
	TargetGuid = FGuid::NewGuid();
}


AGAGridActor* UGATargetComponent::GetGridActor() const
{
	AGAGridActor* Result = GridActor.Get();
	if (Result)
	{
		return Result;
	}
	else
	{
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


void UGATargetComponent::OnRegister()
{
	Super::OnRegister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->RegisterTargetComponent(this);
	}

	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		OccupancyMap = FGAGridMap(Grid, 0.0f);
	}
}

void UGATargetComponent::OnUnregister()
{
	Super::OnUnregister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->UnregisterTargetComponent(this);
	}
}



void UGATargetComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	bool isImmediate = false;

	// update my perception state FSM
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGAPerceptionComponent>> &PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
		for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
		{
			const FTargetView* TargetView = PerceptionComponent->GetTargetView(TargetGuid);
			if (TargetView && (TargetView->Awareness >= 1.0f))
			{
				isImmediate = true;
				break;
			}
		}
	}

	if (isImmediate)
	{
		AActor* Owner = GetOwner();
		LastKnownState.State = GATS_Immediate;

		// REFRESH MY STATE
		LastKnownState.Set(Owner->GetActorLocation(), Owner->GetVelocity());

		// Tell the omap to clear out and put all the probability in the observed location
		OccupancyMapSetPosition(LastKnownState.Position);
	}
	else if (IsKnown())
	{
		LastKnownState.State = GATS_Hidden;
	}

	if (LastKnownState.State == GATS_Hidden)
	{
		OccupancyMapUpdate();
	}

	// As long as I'm known, whether I'm immediate or not, diffuse the probability in the omap

	if (IsKnown())
	{
		OccupancyMapDiffuse();
	}

	if (bDebugOccupancyMap)
	{
		AGAGridActor* Grid = GetGridActor();
		Grid->DebugGridMap = OccupancyMap;
		GridActor->RefreshDebugTexture();
		GridActor->DebugMeshComponent->SetVisibility(true);
	}
}


void UGATargetComponent::OccupancyMapSetPosition(const FVector& Position)
{
	// TODO PART 4

	// We've been observed to be in a given position
	// Clear out all probability in the omap, and set the appropriate cell to P = 1.0
	AGAGridActor* Grid = GetGridActor();
	FCellRef P = Grid->GetCellRef(Position);
	OccupancyMap.ResetData(0.0f);
	OccupancyMap.SetValue(P, 1.0f);
}


void UGATargetComponent::OccupancyMapUpdate()
{
	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		FGAGridMap VisibilityMap(Grid, 0.0f);

		// PART 4

		// STEP 1: Build a visibility map, based on the perception components of the AIs in the world
		// The visibility map is a simple map where each cell is either 0 (not currently visible to ANY perceiver) or 1 (currently visible to one or more perceivers).
		// 

		UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
		if (PerceptionSystem)
		{
			TArray<TObjectPtr<UGAPerceptionComponent>>& PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
			for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
			{
				// Find visible cells for this perceiver.
				// Reminder: Use the PerceptionComponent.VisionParameters when determining whether a cell is visible or not (in addition to a line trace).
				// Suggestion: you might find it useful to add a UGAPerceptionComponent::TestVisibility method to the perception component.

				PerceptionComponent->TestVisibility(Grid, &VisibilityMap);

			}
		}


		// STEP 2: Clear out the probability in the visible cells
		//loop over vmap to reset those cell in omap
		for (int32 X = 0; X < VisibilityMap.XCount; X++) {
			for (int32 Y = 0; Y < VisibilityMap.YCount; Y++)
			{
				FCellRef Cell(X, Y);
				float Value;
				VisibilityMap.GetValue(Cell, Value);
				if (Value > 0.0f) {
					//OccupancyMapSetPosition()
					OccupancyMap.SetValue(Cell, 0.0f);
				}
			}
		}

		float Sum = 0;
		// STEP 3: Renormalize the OMap, so that it's still a valid probability distribution
		// recalculate the sum of all non-zero(non-visible) probabilities, divide each cell by it to renormalize
		for (int32 X = 0; X < OccupancyMap.XCount; X++) {
			for (int32 Y = 0; Y < OccupancyMap.YCount; Y++)
			{
				FCellRef Cell(X, Y);
				float Value;
				OccupancyMap.GetValue(Cell, Value);
				if (Value!= 0.0f) {
					Sum += Value;
				}
			}
		}
		for (int32 X = 0; X < OccupancyMap.XCount; X++) {
			for (int32 Y = 0; Y < OccupancyMap.YCount; Y++)
			{
				FCellRef Cell(X, Y);
				float Value;
				OccupancyMap.GetValue(Cell, Value);
				if (Value != 0.0f) {
					OccupancyMap.SetValue(Cell, Value / Sum);
				}
			}
		}

		// STEP 4: Extract the highest-likelihood cell on the omap and refresh the LastKnownState.
		float Highest = -1.0f;
		FCellRef BestCell;
		//OccupancyMap.GetMaxValue()
		for (int32 X = 0; X < OccupancyMap.XCount; X++) {
			for (int32 Y = 0; Y < OccupancyMap.YCount; Y++)
			{
				FCellRef Cell(X, Y);
				float Value;
				OccupancyMap.GetValue(Cell, Value);
				if (Value != 0.0f && Value > Highest) {
					Highest = Value;
					BestCell = Cell;
				}
			}
		}
		if (BestCell.IsValid()) {
			FVector LastPosition = Grid->GetCellPosition(BestCell);
			LastKnownState.Set(LastPosition, FVector(0.0f));
		}

	}

}


void UGATargetComponent::OccupancyMapDiffuse()
{
	// PART 4
	// Diffuse the probability in the OMAP
	
	const AGAGridActor* Grid = GetGridActor();
	// create a empty buffer map 
	FGAGridMap Buffer(Grid, 0.0f);
	float Alpha = 0.03f;
	for (int32 X = 0; X < OccupancyMap.XCount; X++) {
		for (int32 Y = 0; Y < OccupancyMap.YCount; Y++)
		{
			FCellRef C(X, Y);
			float LeftVal;
			OccupancyMap.GetValue(C, LeftVal);
			float Val = LeftVal;
			for (FCellRef N : GetNeighbors(C, Grid)) {
				//if is diagonal 
				float Diffu;
				
				if (FMath::Abs(C.X - N.X) == 1.0f && FMath::Abs(C.Y - N.Y) == 1.0f) {
					Diffu = Alpha * Val / FMath::Sqrt(2.0f);
				}
				else {
					Diffu = Alpha * Val;
				}
				float NVal;
				Buffer.GetValue(N, NVal);
				Buffer.SetValue(N, Diffu + NVal);
				LeftVal -= Diffu;
			}
			
			float OldBufferVal;
			Buffer.GetValue(C, OldBufferVal);
			Buffer.SetValue(C, OldBufferVal + LeftVal);

		}
	}
	OccupancyMap = Buffer;
}

//finding neighbors
TArray<FCellRef> UGATargetComponent::GetNeighbors(const FCellRef& Cell, const AGAGridActor* Grid) const {

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
