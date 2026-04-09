
#include "GACMMPathComponent.h"
#include "GameAI/CMM/GACMMDataActor.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/NavMovementComponent.h"

UGACMMPathComponent::UGACMMPathComponent(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    PrimaryComponentTick.bCanEverTick = true;
    State = GAPS_None;
    bDestinationValid = false;
    ArrivalDistance = 100.0f;
}


APawn* UGACMMPathComponent::GetOwnerPawn() const
{
    AActor* Owner = GetOwner();
    if (Owner)
    {
        APawn* Pawn = Cast<APawn>(Owner);
        if (Pawn) return Pawn;

        AController* Controller = Cast<AController>(Owner);
        if (Controller) return Controller->GetPawn();
    }
    return nullptr;
}


AGACMMDataActor* UGACMMPathComponent::GetCMMData() const
{
    if (CMMDataActor.Get())
    {
        return CMMDataActor.Get();
    }

    AActor* Found = UGameplayStatics::GetActorOfClass(this, AGACMMDataActor::StaticClass());
    if (Found)
    {
        CMMDataActor = Cast<AGACMMDataActor>(Found);
        return CMMDataActor.Get();
    }

    return nullptr;
}


EGAPathState UGACMMPathComponent::SetDestination(const FVector& DestinationPoint)
{
    Destination = DestinationPoint;
    State = GAPS_Invalid;
    bDestinationValid = true;

    RefreshPath();

    return State;
}


void UGACMMPathComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    if (GetOwnerPawn() == nullptr)
    {
        return;
    }

    if (bDestinationValid)
    {
        RefreshPath();
    }

    if (State == GAPS_Active)
    {
        FollowPath();
    }

    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}


EGAPathState UGACMMPathComponent::RefreshPath()
{
    APawn* Owner = GetOwnerPawn();
    if (Owner == nullptr)
    {
        State = GAPS_Invalid;
        return State;
    }

    FVector StartPoint = Owner->GetActorLocation();
    float DistanceToDestination = FVector::Dist(StartPoint, Destination);

    if (DistanceToDestination <= ArrivalDistance)
    {
        State = GAPS_Finished;
    }
    else
    {
        Steps.Empty();
        State = BackboneAStar(StartPoint);
    }

    // After planning, remove any steps that are behind the agent
    // (closer to start than the agent already is)
    if (State == GAPS_Active && Steps.Num() > 1)
    {
        // Find the step closest to the agent
        float BestDist = FLT_MAX;
        int32 BestIdx = 0;
        for (int32 i = 0; i < Steps.Num(); i++)
        {
            float Dist = FVector::Dist(StartPoint, Steps[i].Point);
            if (Dist < BestDist)
            {
                BestDist = Dist;
                BestIdx = i;
            }
        }

        // Remove everything up to and including the closest step
        // so Steps[0] is the next point AHEAD
        if (BestIdx < Steps.Num() - 1)
        {
            Steps.RemoveAt(0, BestIdx + 1);
        }
    }
    return State;
}


FCellRef UGACMMPathComponent::FindNearestNode(FBackboneGraph& Graph, const FVector& WorldPos) const
{
    // Sort candidates by distance
    struct FCandidate
    {
        FCellRef Cell;
        float Dist;
        bool operator<(const FCandidate& Other) const { return Dist < Other.Dist; }
    };

    TArray<FCandidate> Candidates;
    for (FBackboneNode& Node : Graph.Nodes)
    {
        if (Node.NeighborNodes.Num() == 0) continue;
        float Dist = FVector::Dist2D(WorldPos, Node.WorldPosition);
        Candidates.Add({ Node.Cell, Dist });
    }
    Candidates.Sort();

    // Pick the closest node that has clear line of sight
    UWorld* World = GetWorld();
    for (const FCandidate& C : Candidates)
    {
        FBackboneNode* Node = Graph.FindNode(C.Cell);
        if (!Node) continue;

        FHitResult Hit;
        FCollisionQueryParams Params;
        APawn* Pawn = GetOwnerPawn();
        if (Pawn) Params.AddIgnoredActor(Pawn);

        FVector Start = WorldPos + FVector(0, 0, 50);
        FVector End = Node->WorldPosition + FVector(0, 0, 50);

        bool bBlocked = World->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, Params);

        if (!bBlocked)
        {
            return C.Cell;
        }
    }

    // Fallback: return closest even without LOS
    if (Candidates.Num() > 0)
    {
        return Candidates[0].Cell;
    }

    return FCellRef::Invalid;
}


EGAPathState UGACMMPathComponent::BackboneAStar(const FVector& StartPoint)
{
    AGACMMDataActor* CMMData = GetCMMData();
    if (!CMMData)
    {
        UE_LOG(LogTemp, Warning, TEXT("UGACMMPathComponent - No CMMDataActor found"));
        return GAPS_Invalid;
    }

    const AGAGridActor* Grid = CMMData->GridActor.Get();
    if (!Grid) return GAPS_Invalid;

    FBackboneGraph& Graph = CMMData->Graph;

    FCellRef StartCell = FindNearestNode(Graph, StartPoint);
    FCellRef GoalCell = FindNearestNode(Graph, Destination);

    // ADD THIS BLOCK
    //FBackboneNode* DebugStart = Graph.FindNode(StartCell);
    //FBackboneNode* DebugGoal = Graph.FindNode(GoalCell);
    //UE_LOG(LogTemp, Warning, TEXT("Start:(%d,%d) neighbors:%d  Goal:(%d,%d) neighbors:%d"),
    //    StartCell.X, StartCell.Y,
    //    DebugStart ? DebugStart->NeighborNodes.Num() : -1,
    //    GoalCell.X, GoalCell.Y,
    //    DebugGoal ? DebugGoal->NeighborNodes.Num() : -1);
    // END BLOCK

    if (!StartCell.IsValid() || !GoalCell.IsValid())
    {
        return GAPS_Invalid;
    }

    // Already at goal node
    if (StartCell == GoalCell)
    {
        FBackboneNode* Node = Graph.FindNode(StartCell);
        FPathStep Step;
        Step.Set(Node->WorldPosition, Node->Cell);
        Steps.Add(Step);
        return GAPS_Active;
    }

    FBackboneNode* GoalNode = Graph.FindNode(GoalCell);

    // A* data structures
    TArray<FNodeCost> OpenList;
    TSet<FCellRef> ClosedSet;
    TMap<FCellRef, float> GCost;
    TMap<FCellRef, FCellRef> CameFrom;

    // Initialize
    FBackboneNode* StartNode = Graph.FindNode(StartCell);
    GCost.Add(StartCell, 0.0f);
    float H = FVector::Dist(StartNode->WorldPosition, GoalNode->WorldPosition);
    OpenList.HeapPush(FNodeCost(StartCell, H));

    while (OpenList.Num() > 0)
    {
        FNodeCost Current;
        OpenList.HeapPop(Current);

        if (ClosedSet.Contains(Current.Cell)) continue;
        ClosedSet.Add(Current.Cell);

        // Reached goal — reconstruct path
        if (Current.Cell == GoalCell)
        {
            // Trace back node sequence
            TArray<FCellRef> NodePath;
            FCellRef Trace = GoalCell;

            while (!(Trace == StartCell))
            {
                NodePath.Add(Trace);
                Trace = CameFrom[Trace];
            }
            NodePath.Add(StartCell);
            Algo::Reverse(NodePath);

            // Build Steps from edge path points
            for (int32 i = 0; i < NodePath.Num() - 1; i++)
            {
                FBackboneNode* FromNode = Graph.FindNode(NodePath[i]);
                FCellRef ToCell = NodePath[i + 1];

                for (int32 j = 0; j < FromNode->NeighborNodes.Num(); j++)
                {
                    if (FromNode->NeighborNodes[j] == ToCell)
                    {
                        const FBackboneEdge& EdgeData = FromNode->NeighborEdge[j];

                        // Skip first point after first edge to avoid duplicates
                        //int32 StartIdx = (i == 0) ? 0 : 1;
                        for (int32 k = 1; k < EdgeData.PathPoints.Num(); k++)
                        {
                            FPathStep Step;
                            // Convert world pos back to cell ref for FPathStep
                            FCellRef CellRef = Grid->GetCellRef(EdgeData.PathPoints[k]);
                            Step.Set(EdgeData.PathPoints[k], CellRef);
                            Steps.Add(Step);
                        }
                        break;
                    }
                }
            }

            //UE_LOG(LogTemp, Log, TEXT("BackboneAStar - Path found: %d nodes, %d steps"),NodePath.Num(), Steps.Num());

            // add agent's actual position as first step
         /*   FPathStep FirstStep;
            FCellRef StartCellRef = Grid->GetCellRef(StartPoint);
            FirstStep.Set(StartPoint, StartCellRef);
            Steps.Insert(FirstStep, 0);*/
            UE_LOG(LogTemp, Warning, TEXT("Backbone A*: expanded %d of %d nodes"),
                ClosedSet.Num(), Graph.Nodes.Num());

            return GAPS_Active;
        }

        // Expand neighbors
        FBackboneNode* CurrentNode = Graph.FindNode(Current.Cell);
        float CurrentG = GCost[Current.Cell];

        for (int32 i = 0; i < CurrentNode->NeighborNodes.Num(); i++)
        {
            FCellRef NeighborCell = CurrentNode->NeighborNodes[i];
            if (ClosedSet.Contains(NeighborCell)) continue;

            float NewG = CurrentG + CurrentNode->NeighborCosts[i];
            float* ExistingG = GCost.Find(NeighborCell);

            if (!ExistingG || NewG < *ExistingG)
            {
                GCost.Add(NeighborCell, NewG);
                CameFrom.Add(NeighborCell, Current.Cell);

                FBackboneNode* NeighborNode = Graph.FindNode(NeighborCell);
                float NewH = FVector::Dist(NeighborNode->WorldPosition, GoalNode->WorldPosition);
                OpenList.HeapPush(FNodeCost(NeighborCell, NewG + NewH));
            }
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("BackboneAStar - No path found. Expanded %d of %d nodes"),
        ClosedSet.Num(), Graph.Nodes.Num());

    //// Log all reachable nodes
    //for (const FCellRef& Cell : ClosedSet)
    //{
    //    UE_LOG(LogTemp, Warning, TEXT("  Reachable: (%d,%d)"), Cell.X, Cell.Y);
    //}
    return GAPS_Invalid;
}


void UGACMMPathComponent::FollowPath()
{
    APawn* Owner = GetOwnerPawn();
    if (Owner == nullptr) return;

    FVector StartPoint = Owner->GetActorLocation();

    check(State == GAPS_Active);
    check(Steps.Num() > 0);

    // Follow the first step, same as GAPathComponent
    FVector V = Steps[0].Point - StartPoint;
    V.Normalize();

    UNavMovementComponent* MovementComponent = Owner->FindComponentByClass<UNavMovementComponent>();
    if (MovementComponent)
    {
        MovementComponent->RequestPathMove(V);
    }
}


void UGACMMPathComponent::DebugDrawPath()
{
    float ZOffset = 80.0f;

    for (int32 i = 0; i < Steps.Num(); i++)
    {
        FVector Pos = Steps[i].Point + FVector(0, 0, ZOffset);

        DrawDebugSphere(GetWorld(), Pos, 15.0f, 6, FColor::Orange, true, -1.0f);

        if (i < Steps.Num() - 1)
        {
            FVector Next = Steps[i + 1].Point + FVector(0, 0, ZOffset);
            DrawDebugLine(GetWorld(), Pos, Next, FColor::Orange, true, -1.0f, 0, 4.0f);
        }
    }

    if (Steps.Num() > 0)
    {
        DrawDebugSphere(GetWorld(), Steps[0].Point + FVector(0, 0, ZOffset), 30.0f, 8, FColor::Green, true, -1.0f);
        DrawDebugSphere(GetWorld(), Steps.Last().Point + FVector(0, 0, ZOffset), 30.0f, 8, FColor::Red, true, -1.0f);
    }
}