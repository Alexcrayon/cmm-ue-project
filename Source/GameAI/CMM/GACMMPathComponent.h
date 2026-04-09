
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GACMMDataActor.h"
#include "GameAI/Pathfinding/GAPathComponent.h"
#include "GACMMPathComponent.generated.h"

struct FNodeCost
{
    FCellRef Cell;
    float Cost;

    FNodeCost() : Cell(FCellRef::Invalid), Cost(FLT_MAX) {}
    FNodeCost(FCellRef InCell, float InCost) : Cell(InCell), Cost(InCost) {}

    bool operator<(const FNodeCost& Other) const
    {
        return Cost < Other.Cost;
    }
};

UCLASS(BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class UGACMMPathComponent : public UActorComponent
{
    GENERATED_UCLASS_BODY()

public:
    UPROPERTY(BlueprintReadOnly)
    TEnumAsByte<EGAPathState> State;

    UPROPERTY(BlueprintReadWrite)
    TArray<FPathStep> Steps;

    UPROPERTY(BlueprintReadOnly)
    FVector Destination;

    UPROPERTY(BlueprintReadOnly)
    bool bDestinationValid;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float ArrivalDistance;

    UFUNCTION(BlueprintCallable)
    EGAPathState SetDestination(const FVector& DestinationPoint);

    UFUNCTION(BlueprintCallable, BlueprintPure)
    APawn* GetOwnerPawn() const;

    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    UFUNCTION(BlueprintCallable)
    void DebugDrawPath();

private:
    UPROPERTY()
    mutable TSoftObjectPtr<AGACMMDataActor> CMMDataActor;

    AGACMMDataActor* GetCMMData() const;

    EGAPathState RefreshPath();
    EGAPathState BackboneAStar(const FVector& StartPoint);
    FCellRef FindNearestNode(FBackboneGraph& Graph, const FVector& WorldPos) const;

    void FollowPath();
};