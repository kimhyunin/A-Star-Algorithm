using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

class Pos
{
    public Pos(int x, int z) { X = x; Z = z; }
    public int X;
    public int Z;
}
public class AstarPathPlaning: MonoBehaviour
{
    struct Node : IComparable<Node>{
    public int F;
    public int G;
    public int X;
    public int Z;

    public int CompareTo(Node other){
        if (F == other.F)
            return 0;
        return F < other.F ? 1 : -1;
        }
    }
    // 시작 위치
    public int PosX { get; private set; }
    public int PosZ { get; private set; }
    // 목적지
    public int DestX {get; private set; }
    public int DestZ {get; private set; }
    private ChessPiece _chessPiece;
    List<Pos> _point = new List<Pos>();
    List<Vector3> path = new List<Vector3>();
    GameObject[,] _board;
    /// <summary>
    /// 시작 위치, 목표 위치 세팅 후 A* 알고리즘 실행
    /// </summary>
    /// <param name="chessPiece">선택된 체스말</param>
    /// <param name="start">시작 위치 배열</param>
    /// <param name="end">목표 위치 배열</param>
    /// <param name="board">현재 체스판 상태를 담은 배열</param>
    /// <returns> A* 알고리즘을 통해 얻은 최적의 경로 List<Vector3></returns>
    public List<Vector3> Init(GameObject chessPiece, int[] start, int[] end, GameObject[,] board){
        // 초기화
        Initialization();
        _chessPiece = chessPiece.GetComponent<ChessPiece>();
        // 시작위치
        PosX = start[0];
        PosZ = start[1];
        // 끝지점
        DestX = end[0];
        DestZ = end[1];
        _board = board;

        Debug.Log("PosX :"+ PosX + " PosZ :" + PosZ + "Position : " + _board + " End : " + end[0] + " End 1: " + end[1]);
        // 경로찾기 시작
        StartCoroutine(Astar());
        return path;
    }
    /// <summary>
    /// A* 알고리즘 시작시 리스트를 초기화 시켜줌
    /// </summary>
    public void Initialization(){
        _point.Clear();
        path.Clear();
    }

    IEnumerator Astar(){
                                // U   D   L  R  UR  UL  DR  DL
        int[] deltaX = new int[] { 0,  0, -1, 1, 1, -1,  1, -1};
        int[] deltaZ = new int[] { 1, -1,  0, 0, 1,  1, -1, -1};

        int[] cost = new int[]   { 10,  10,  10, 10, 14,  14,  14 , 14};
        // 점수 매기기
        // F = G + H
        // F = 최종 점수 (작을 수록 좋음, 경로에 따라 달라짐) 출발 지점에서 목적지까지의 총 Cost 합
        // G = 시작점에서 해당 좌표까지 이동하는데 드는 비용 (작을 수록 좋음, 경로에 따라 달라짐) 현재 노드에서 출발 지점까지의 총 Cost
        // H = 목적지에서 얼마나 가까운지 (작을 수록 좋음, 고정) 휴리스틱, 현재 노드에서 목적지까지의 추정거리

        // (x, z) 이미 방문했는지 여부 (방문 = closed 상태)
        bool[,] closed = new bool[_board.GetLength(0),_board.GetLength(0)]; // CloseList

        // (x, z) 한 번이라도 발견했는지 여부 (예약 = open 상태)
        int[,] open = new int[_board.GetLength(0),_board.GetLength(0)]; // openList

        // openList에 F 값을 최대값을 넣음
        for(int x = 0; x < _board.GetLength(0); x++){
            for(int z = 0; z < _board.GetLength(0); z++){
                open[x, z] = Int32.MaxValue;
            }
        }
        // 방문한 지점을 저장하는 배열
        Pos[,] parent = new Pos[_board.GetLength(0), _board.GetLength(0)];
        // 우선순위 큐를 이용하여 openList에 있는 정보들 중 , 가장 좋은 후보를 뽑기위한 도구
        PriorityQueue<Node> pq = new PriorityQueue<Node>();

        // 시작점 (예약진행)
        open[PosX, PosZ] = 10 * (Math.Abs(DestX - PosX) + Math.Abs(DestZ - PosZ));
        pq.Push(new Node() {F = 10 * (Math.Abs(DestX - PosX) + Math.Abs(DestZ - PosZ)), G = 0, X = PosX, Z = PosZ});
        parent[PosX,PosZ] = new Pos(PosX, PosZ);

        while(pq.Count > 0){
            // 제일 좋은 후보를 가져온다.
            Node node = pq.Pop();
            // 이미 방문한 좌표이면 스킵
            if(closed[node.X, node.Z])
                continue;
            // 도착한 좌표는 방문을 했기에 방문처리
            closed[node.X, node.Z] = true;
            Debug.Log("I :: nodeX : " + node.X + " nodeZ : " + node.Z + " DestX: " + DestX + " DeseZ :" + DestZ);
            // 도착한 좌표가 목적지이면 경로 탐색 중지
            if(node.X == DestX && node.Z == DestZ)
                break;

            // 현재 좌표에서 U   D   L  R  UR  UL  DR  DL 지점을 탐색하여 이동가능한 좌표인지 확인 후 예약 진행
            for(int i = 0; i < deltaX.Length; i++){
                int nextX = node.X + deltaX[i];
                int nextZ = node.Z + deltaZ[i];

                // 해당 좌표가 맵을 벗어난 경우 스킵
                if(nextX <= 0 || nextX >= _board.GetLength(0) || nextZ <= 0 || nextZ >= _board.GetLength(0))
                    continue;
                // 해당 좌표에 다른 체스말이 존재하고, 최종 목적지가 아니면 스킵
                if(_board[nextX, nextZ] != null){
                    if(nextX != DestX || nextZ != DestZ){
                        continue;
                    }
                }
                // 한번 방문한 좌표이면 스킵
                if(closed[nextX, nextZ])
                    continue;

                Debug.Log("O :: nextX : " + nextX + " nextZ : " + nextZ);
                // 비용 계산
                int g = node.G + cost[i];
                int h = 10 *(Math.Abs(DestX - PosX) + Math.Abs(DestZ - PosZ));
                int f = g + h;
                // 상.하.좌.우.대각선 다른 방향에서 이미 F값이 낮은거를 찾았으면 스킵
                if(open[nextX, nextZ] < f)
                    continue;

                // 상.하.좌.우.대각선 방향에서 찾은 F값중 제일 작은 F값만 저장
                // 예약 진행
                open[nextX, nextZ] = f;
                pq.Push(new Node() {F = g+h, G = g, X = nextX, Z = nextZ});
                parent[nextX, nextZ] = new Pos(node.X, node.Z);
            }
        }
        yield return StartCoroutine(CalculatePath(parent));
    }
    /// <summary>
    /// 도착지점에서 부터 시작점까지 역으로 추적하여 경로생성
    /// </summary>
    /// <param name="parent">A* 알고리즘에서 얻은 예약이 된 배열</param>
    IEnumerator CalculatePath(Pos[,] parent){
        int x = DestX;
        int z = DestZ;
        // 경로를 만들지 못하였으면 스킵
        if(parent[x, z] != null){
            // 역추적 시작
            while(parent[x, z].X != x || parent[x, z].Z != z){
                _point.Add(new Pos(x,z));
                Pos pos = parent[x,z];
                x = pos.X;
                z = pos.Z;
            }
            // 경로가 도착지 -> 시작점으로 되어있기
            // Reverse를 통해 배열을 뒤집기
            _point.Reverse();
            // 얻은 경로를 List<Vector3> 타입으로 바꿔 값을 반환
            for(int i = 0; i < _point.Count; i++){
                path.Add(new Vector3(_point[i].X, 0f , _point[i].Z));
            }
        }
        yield return null;
    }
}
