// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameAI/Grid/GAGridMap.h"
#include "GameAI/Grid/GAGridActor.h"
#include "GACMMDataActor.generated.h"

UCLASS(BlueprintType, Blueprintable)
class GAMEAI_API AGACMMDataActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AGACMMDataActor();
	FGAGridMap DistanceTransform;
	UFUNCTION(BlueprintCallable)
	void BuildDistanceTransform();

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	mutable TSoftObjectPtr<AGAGridActor> GridActor;


	UFUNCTION(BlueprintCallable)
	const AGAGridActor* GetGridActor() const;

	bool IsEdgeCell(const AGAGridActor* Grid, const FCellRef& Cell ) const;

	struct FCellDist
	{
		FCellRef Cell;
		float Distance;
	};

	TArray<FCellRef> GetNeighbors(const FCellRef& Cell, const AGAGridActor* Grid) const;

	UFUNCTION(BlueprintCallable)
	void DebugDrawDistanceTransform();
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

};
