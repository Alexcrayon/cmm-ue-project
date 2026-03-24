#include "GAPerceptionComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GAPerceptionSystem.h"
#include "GameAI/Grid/GAGridActor.h"

UGAPerceptionComponent::UGAPerceptionComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;
}


void UGAPerceptionComponent::OnRegister()
{
	Super::OnRegister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->RegisterPerceptionComponent(this);
	}
}

void UGAPerceptionComponent::OnUnregister()
{
	Super::OnUnregister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->UnregisterPerceptionComponent(this);
	}
}


APawn* UGAPerceptionComponent::GetOwnerPawn() const
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



// Returns the Target this AI is attending to right now.

UGATargetComponent* UGAPerceptionComponent::GetCurrentTarget() const
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);

	if (PerceptionSystem && PerceptionSystem->TargetComponents.Num() > 0)
	{
		UGATargetComponent* TargetComponent = PerceptionSystem->TargetComponents[0];
		if (TargetComponent->IsKnown())
		{
			return PerceptionSystem->TargetComponents[0];
		}
	}

	return NULL;
}

bool UGAPerceptionComponent::HasTarget() const
{
	return GetCurrentTarget() != NULL;
}


bool UGAPerceptionComponent::GetCurrentTargetState(FTargetState& TargetStateOut, FTargetView& TargetViewOut) const
{
	UGATargetComponent* Target = GetCurrentTarget();
	if (Target)
	{
		const FTargetView* TargetView = TargetMap.Find(Target->TargetGuid);
		if (TargetView)
		{
			TargetStateOut = Target->LastKnownState;
			TargetViewOut = *TargetView;
			return true;
		}

	}
	return false;
}


void UGAPerceptionComponent::GetAllTargetStates(bool OnlyKnown, TArray<FTargetState>& TargetStatesOut, TArray<FTargetView>& TargetViewsOut) const
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGATargetComponent>>& TargetComponents = PerceptionSystem->GetAllTargetComponents();
		for (UGATargetComponent* TargetComponent : TargetComponents)
		{
			const FTargetView* TargetView = TargetMap.Find(TargetComponent->TargetGuid);
			if (TargetView)
			{
				if (!OnlyKnown || TargetComponent->IsKnown())
				{
					TargetStatesOut.Add(TargetComponent->LastKnownState);
					TargetViewsOut.Add(*TargetView);
				}
			}
		}
	}
}


void UGAPerceptionComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateAllTargetViews();
}


void UGAPerceptionComponent::UpdateAllTargetViews()
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGATargetComponent>>& TargetComponents = PerceptionSystem->GetAllTargetComponents();
		for (UGATargetComponent* TargetComponent : TargetComponents)
		{
			UpdateTargetView(TargetComponent);
		}
	}
}

void UGAPerceptionComponent::UpdateTargetView(UGATargetComponent* TargetComponent)
{
	// REMEMBER: the UGAPerceptionComponent is going to be attached to the controller, not the pawn. So we call this special accessor to 
	// get the pawn that our controller is controlling
	APawn* OwnerPawn = GetOwnerPawn();
	if (OwnerPawn == NULL)
	{
		return;
	}

	FTargetView *TargetView = TargetMap.Find(TargetComponent->TargetGuid);
	if (TargetView == NULL)		// If we don't already have a target data for the given target component, add it
	{
		FTargetView NewTargetView;
		FGuid TargetGuid = TargetComponent->TargetGuid;
		TargetView = &TargetMap.Add(TargetGuid, NewTargetView);
	}

	if (TargetView)
	{
		// PART 3
		// 
		// - Update TargetView->bClearLOS
		//		Use this.VisionParameters to determine whether the target is within the vision cone or not 
		//		(and ideally do so before you cast a ray towards it)
		// - Update TargetView->Awareness
		//		On ticks when the AI has a clear LOS, the Awareness should grow
		//		On ticks when the AI does not have a clear LOS, the Awareness should decay
		//
		// Awareness should be clamped to the range [0, 1]
		// You can add parameters to the UGAPerceptionComponent to control the speed at which awareness rises and falls
		
		FVector CurrLoc = OwnerPawn->GetActorLocation();
		FVector CurrDir = OwnerPawn->GetActorForwardVector();
		CurrDir.Normalize();

		

		FVector TargetLoc = TargetComponent->GetOwner()->GetActorLocation();

		FVector TargetVec = TargetLoc - CurrLoc;
		double Dist = TargetVec.Size(); 

		TargetVec.Normalize();

		float HalfCone = FMath::DegreesToRadians(VisionParameters.VisionAngle / 2.0f);

		if (Dist <= this->VisionParameters.VisionDistance && FVector::DotProduct(CurrDir, TargetVec) >= cos(HalfCone) ) {
			//is within the vision cone
			//casting a ray
			UWorld* World = GetWorld();
			FHitResult HitResult;
			FCollisionQueryParams Params;
			FVector Start = CurrLoc;		
			FVector End = TargetLoc;		
			Start.Z += 50.0f;								
			
			Params.AddIgnoredActor(TargetComponent->GetOwner());			
			Params.AddIgnoredActor(OwnerPawn);		
			bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);

			if (bHitSomething) {
			
				TargetView->bClearLos = false;

			}
			else
			{
				TargetView->bClearLos = true;
			}
		}
		else {
			TargetView->bClearLos = false;
		}

		//update awareness
		if (TargetView->bClearLos) {
			TargetView->Awareness += 0.05f;

			TargetView->Awareness = FMath::Clamp(TargetView->Awareness, 0, 1);

		}
		else {
			TargetView->Awareness -= 0.05f;
			TargetView->Awareness = FMath::Clamp(TargetView->Awareness, 0, 1);
		}
		
		//for debugging
		/*UE_LOG(LogTemp, Warning, TEXT("Current Awareness: %f"), TargetView->Awareness);
		UE_LOG(LogTemp, Warning, TEXT("LOS: %d"), TargetView->bClearLos);
		UE_LOG(LogTemp, Warning, TEXT("Distance: %f"), Dist);
		*/
	}
}


const FTargetView* UGAPerceptionComponent::GetTargetView(FGuid TargetGuid) const
{
	return TargetMap.Find(TargetGuid);
}



void UGAPerceptionComponent::TestVisibility(const AGAGridActor* Grid, FGAGridMap* VMap) {
	APawn* Pawn = this->GetOwnerPawn();
	if (Pawn == NULL) {
		return;
	}
	FVector PawnLoc = Pawn->GetActorLocation();
	FVector FacingDir = Pawn->GetActorForwardVector();

	float HalfCone = FMath::DegreesToRadians(this->VisionParameters.VisionAngle / 2.0f);
	for (int32 X = 0; X < Grid->XCount; X++)
	{
		for (int32 Y = 0; Y < Grid->YCount; Y++)
		{
			FCellRef CellRef(X, Y);
			FVector CellPos = Grid->GetCellPosition(CellRef);
			FVector ToCell = CellPos - PawnLoc;
			float Dist = ToCell.Size();
			ToCell.Normalize();
			float Dot = FVector::DotProduct(FacingDir, ToCell);
			//vision cone check
			if (Dist <= this->VisionParameters.VisionDistance && Dot >= FMath::Cos(HalfCone))
			{
				//line of sight check
				UWorld* World = GetWorld();
				FHitResult HitResult;
				FCollisionQueryParams Params;
				FVector Start = PawnLoc;
				FVector End = CellPos;
				Start.Z += 50.0f;

				Params.AddIgnoredActor(this->GetOwner());
				Params.AddIgnoredActor(Pawn);
				bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);
			
				if (!bHitSomething) {
					/*return true;*/
					VMap->SetValue(CellRef, 1.0f);
				}
			}
		}
	}
}
