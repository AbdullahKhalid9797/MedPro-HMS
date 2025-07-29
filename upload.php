<?php
require_once "db.php";
date_default_timezone_set("Asia/Karachi");

$data = file_get_contents("php://input");
parse_str($data, $params);
$timestamp = date('Y-m-d H:i:s');

if (!isset($params['P']) || !isset($params['S'])) {
    http_response_code(400);
    echo "Missing Profile ID or Sensor ID.";
    exit;
}

$pid = intval($params['P']);
$sid = intval($params['S']); // Still useful to tag sensor source

$response = [];

try {
    // --- Temperature (T) ---
    if (isset($params['T'])) {
        $temp = floatval($params['T']);
        $stmt = $pdo->prepare("INSERT INTO Temp (ID, SID, Temperature, TimeStamp) VALUES (?, ?, ?, ?)");
        $stmt->execute([$pid, $sid, $temp, $timestamp]);
        $response[] = "Temperature stored";
    }

    // --- Heart Rate (H) or SpO2 (O) ---
    if (isset($params['H']) || isset($params['O'])) {
        $hr = isset($params['H']) ? intval($params['H']) : null;
        $spo2 = isset($params['O']) ? intval($params['O']) : null;
        $stmt = $pdo->prepare("INSERT INTO PulOxym (ID, SID, HeartRate, SpO2, TimeStamp) VALUES (?, ?, ?, ?, ?)");
        $stmt->execute([$pid, $sid, $hr, $spo2, $timestamp]);
        $response[] = "PulOxym stored";
    }

    // --- ECG (E) ---
    if (isset($params['E'])) {
        $ecg = intval($params['E']);
        $stmt = $pdo->prepare("INSERT INTO ECG (ID, SID, EValue, TimeStamp) VALUES (?, ?, ?, ?)");
        $stmt->execute([$pid, $sid, $ecg, $timestamp]);
        $response[] = "ECG stored";
    }

    if (empty($response)) {
        echo "No recognized sensor values found.";
    } else {
        echo implode(" | ", $response);
    }

} catch (Exception $e) {
    http_response_code(500);
    echo "Database error: " . $e->getMessage();
}
?>

