using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine.SceneManagement;
using Unity.VisualScripting;

/*
    CubeとPlaneの1辺の長さ(初期値)は
    Cube:1m
    Plane:10m
*/

[System.Serializable]
public class AxleInfo {
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}

// Rescue_Robot_Agent
public class Rescue_Robot_Agent_Road : Agent
{

    public GameObject goal; // GoalのGameObject
    Rigidbody rBody; // Rescue_Robot_AgentのRigidbody

    public List<AxleInfo> axleInfos; 
    public float maxMotorTorque; // 最大モータートルク
    public float maxSteeringAngle; // 最大ステアリング角

    float stageNumber = 0.0f; // エピソード開始時のステージ番号を取得

    int actionCount = 10; // 行動の回数をカウント

    // episodeCount1とepisodeCount2は同じ値にすること
    public int episodeCount1 = 100; // 指定したエピソード回数をカウント（評価用）
    public int episodeCount2 = 100; // ゴール到達率の分母（評価用）
    int goalCount = 0; // ゴール到達回数をカウント（評価用）
    
    // 対応する視覚的なホイールを見つける
    // Transform を正しく適用
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0) {
            return;
        }
     
        Transform visualWheel = collider.transform.GetChild(0);
     
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
     
        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }

    // ゲームオブジェクト生成時に呼ばれる
    public override void Initialize()
    {
        // Rescue_Robot_AgentのRigidBodyの参照の取得
        this.rBody = GetComponent<Rigidbody>();
    }

    // エピソード開始時に呼ばれる
    public override void OnEpisodeBegin()
    {

        if (episodeCount1 == 0){
            evaluation(goalCount, episodeCount2);
        }

        episodeCount1 -= 1;

        // エピソード開始時のステージ番号を取得
        stageNumber = Academy.Instance.EnvironmentParameters.GetWithDefault("stage_number", 0.0f);

        // 現在のステージの学習が終わったらアプリケーションを終了
        if(SceneManager.GetActiveScene().name == "Sotsuken3-Stage1" && stageNumber > 1.0f){
            Application.Quit();
        }
        if(SceneManager.GetActiveScene().name == "Sotsuken3-Stage2" && stageNumber > 2.0f){
            Application.Quit();
        }
        if(SceneManager.GetActiveScene().name == "Sotsuken3-Stage3" && stageNumber > 3.0f){
            Application.Quit();
        }
        if(SceneManager.GetActiveScene().name == "Sotsuken3-Stage4" && stageNumber > 4.0f){
            Application.Quit();
        }
        if(SceneManager.GetActiveScene().name == "Sotsuken3-Stage5" && stageNumber > 5.0f){
            Application.Quit();
        }

        // Rescue_Robot_Agentの位置と速度をリセット
        this.rBody.angularVelocity = Vector3.zero;
        this.rBody.velocity = Vector3.zero;
        this.transform.localPosition = new Vector3(0.0f, 0.1f, 0.1f);
        this.transform.rotation = Quaternion.Euler(0.0f, 0.0f, 0.0f);

        // 障害物の位置をランダムにリセット(Stage2以降)
        if(Mathf.Approximately(stageNumber, 2.0f)){
            GameObject[] rubbles = GameObject.FindGameObjectsWithTag ("Rubble");
            foreach(GameObject rubble in rubbles){
                rubble.transform.localPosition = new Vector3(Random.value * 2.7f - 0.375f, 0.0f, Random.value * 3.0f + 0.5f);
            }
        }
        if(Mathf.Approximately(stageNumber, 3.0f)){
            GameObject[] rubbles = GameObject.FindGameObjectsWithTag ("Rubble");
            foreach(GameObject rubble in rubbles){
                rubble.transform.localPosition = new Vector3(Random.value * 2.7f - 0.375f, 0.0f, Random.value * 3.0f + 0.5f);
            }
        }
        if(Mathf.Approximately(stageNumber, 4.0f)){
            GameObject[] rubbles = GameObject.FindGameObjectsWithTag ("Rubble");
            foreach(GameObject rubble in rubbles){
                rubble.transform.localPosition = new Vector3(Random.value * 2.7f - 0.375f, 0.0f, Random.value * 3.0f + 0.5f);
            }
        }
        if(Mathf.Approximately(stageNumber, 5.0f)){
            GameObject[] rubbles = GameObject.FindGameObjectsWithTag ("Rubble");
            foreach(GameObject rubble in rubbles){
                rubble.transform.localPosition = new Vector3(Random.value * 2.7f - 0.375f, 0.0f, Random.value * 3.0f + 0.5f);
            }
        }

    }

    // 観察取得時に呼ばれる
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(goal.transform.localPosition.x); //TargetのX座標
        sensor.AddObservation(goal.transform.localPosition.z); //TargetのZ座標
        sensor.AddObservation(this.transform.localPosition.x); ; //Rescue_Robot_AgentのX座標
        sensor.AddObservation(this.transform.localPosition.z); ; //Rescue_Robot_AgentのZ座標
        sensor.AddObservation(rBody.velocity.x); //Rescue_Robot_AgentのX速度
        sensor.AddObservation(rBody.velocity.z); //Rescue_Robot_AgentのZ速度
    }

    // 行動決定時に呼ばれる
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {

        actionCount -= 1;

        // Rescue_Robot_Agentに力を加える
        Vector3 controlSignal = Vector3.zero;

        // ブレーキを解除
        axleInfos[1].leftWheel.brakeTorque = 0f;
        axleInfos[1].rightWheel.brakeTorque = 0f;

        // Discreteの場合
        // z軸が前進(1)後退(-1)、x軸がハンドル操作(左:-1、右:1)
        int action = actionBuffers.DiscreteActions[0];

        // 前進のみ
        if (action == 0) {
            controlSignal.z = 1.0f;
        }
        // 前進+左
        if (action == 1) {
            controlSignal.z = 1.0f;
            controlSignal.x = -1.0f;
        }
        // 前進+右
        if (action == 2) {
            controlSignal.z = 1.0f;
            controlSignal.x = 1.0f;
        }
        // 後退のみ
        if (action == 3) {
            controlSignal.z = -1.0f;
        }
        // 後退+左
        if (action == 4) {
            controlSignal.z = -1.0f;
            controlSignal.x = -1.0f;
        }
        // 後退+右
        if (action == 5) {
            controlSignal.z = -1.0f;
            controlSignal.x = 1.0f;
        }
        // 停止
        if(action == 6){
            axleInfos[1].leftWheel.brakeTorque = 100f;
            axleInfos[1].rightWheel.brakeTorque = 100f;
            this.rBody.velocity = Vector3.zero;
        }

        // トルクをかける方向とハンドルを切る方向を決定
        float motor = maxMotorTorque * controlSignal.z;
        float steering = maxSteeringAngle * controlSignal.x;
     
        foreach (AxleInfo axleInfo in axleInfos) {
            // ハンドル操作
            if (axleInfo.steering) {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            // モーター操作
            if (axleInfo.motor) {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
            // ステアリング・モーター操作に対応したアニメーション処理
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }

        // Rescue_Robot_AgentがGoalにたどり着いた時
        float distanceToTarget = Vector3.Distance(this.transform.localPosition, goal.transform.localPosition);
        if(distanceToTarget < 0.3f)
        {
            goalCount += 1;
            AddReward(1.0f); // 報酬を+1点加算
            EndEpisode();
        }
        else if(actionCount < 1)
        {
            actionCount = 10;
            AddReward(distanceToTarget * -0.0001f); // 災害ロボットの現在地がゴール地点から離れているほど減点
        }

        // Rescue_Robot_Agentが道路から落下した時
        if(this.transform.localPosition.y < 0)
        {
            AddReward(-0.5f); // 報酬を-0.5点加算
            EndEpisode();
        }
    }

    // Rescue_Robot_Agentが瓦礫に衝突したとき
    void OnCollisionEnter(Collision collision)
    {
        if(collision.gameObject.name.Contains("Cube"))
        {
            AddReward(-0.1f); // 報酬を-0.1点加算
        }
    }

    // ヒューリスティックモードの行動決定時に呼ばれる
    public override void Heuristic(in ActionBuffers actionBuffers)
    {

        // Discreteの場合
        var actionsOut = actionBuffers.DiscreteActions;
        actionsOut[0] = 6;

        if (Input.GetKey(KeyCode.Alpha1)) actionsOut[0] = 0;
        if (Input.GetKey(KeyCode.Alpha2)) actionsOut[0] = 1;
        if (Input.GetKey(KeyCode.Alpha3)) actionsOut[0] = 2;
        if (Input.GetKey(KeyCode.Alpha4)) actionsOut[0] = 3;
        if (Input.GetKey(KeyCode.Alpha5)) actionsOut[0] = 4;
        if (Input.GetKey(KeyCode.Alpha6)) actionsOut[0] = 5;
        if (Input.GetKey(KeyCode.Alpha7)) actionsOut[0] = 6;

    }

    // ゴール到達率を算出する
    public void evaluation(int goalCount, int episodeCount2)
    {
        float successRate = 0.0f;

        successRate = ((float)goalCount / (float)episodeCount2) * 100.0f;

        Debug.Log("到達率[％]：" + successRate);
    }
}