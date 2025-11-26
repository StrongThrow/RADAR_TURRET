<?php
// /api/radar_current.php
header('Content-Type: application/json; charset=utf-8');
// 프론트가 다른 호스트면 CORS 허용이 필요할 수 있음:
// header('Access-Control-Allow-Origin: *');

date_default_timezone_set('Asia/Seoul');

$dsn  = 'mysql:host=localhost;dbname=iotdb;charset=utf8mb4';
$user = 'iot';
$pass = 'pwiot';

try {
    $pdo = new PDO($dsn, $user, $pass, [
        PDO::ATTR_ERRMODE            => PDO::ERRMODE_EXCEPTION,
        PDO::ATTR_DEFAULT_FETCH_MODE => PDO::FETCH_ASSOC,
    ]);

    $stmt = $pdo->query("SELECT x, y, ts FROM radar_current WHERE id = 1");
    $row  = $stmt->fetch();

    if (!$row) {
        echo json_encode(['ok' => false, 'err' => 'no_row']);
        exit;
    }

    echo json_encode([
        'ok' => true,
        'x'  => (float)$row['x'],
        'y'  => (float)$row['y'],
        'ts' => $row['ts'],     // 예: "2025-08-24 17:12:00"
    ]);
} catch (Throwable $e) {
    http_response_code(500);
    echo json_encode(['ok' => false, 'err' => $e->getMessage()]);
}
