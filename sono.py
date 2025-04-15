import cv2
import mediapipe as mp
import serial
import time

arduino = serial.Serial('COM5', 9600, timeout=1)
time.sleep(2)

mp_face = mp.solutions.face_mesh
face_mesh = mp_face.FaceMesh(max_num_faces=1)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

piscadas = 0
frames_fechado = 0
boca_abrindo = False
tempo_boca_aberta = 0
buzzer_ativado = False

LIMIAR_FECHAR = 0.38    
LIMIAR_ABRIR = 0.35
LIMIAR_COCHILO = 30
LIMIAR_ABERTO = 0.05

def boca_aberta(pontos):
    ponto_sup = pontos[13]
    ponto_inf = pontos[14]
    distancia_boca = abs(ponto_sup.y - ponto_inf.y)
    return distancia_boca > LIMIAR_ABERTO

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    resultados = face_mesh.process(rgb)

    if resultados.multi_face_landmarks:
        pontos = resultados.multi_face_landmarks[0].landmark

        mp_draw.draw_landmarks(
            frame,
            resultados.multi_face_landmarks[0],
            mp_face.FACEMESH_TESSELATION,
            landmark_drawing_spec=mp_draw.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1),
            connection_drawing_spec=mp_draw.DrawingSpec(color=(0, 255, 0), thickness=1)
        )

        h = abs(pontos[33].x - pontos[133].x)
        v1 = abs(pontos[159].y - pontos[145].y)
        v2 = abs(pontos[158].y - pontos[153].y)
        ear = (v1 + v2) / (2.0 * h)

        cv2.putText(frame, f"EAR: {ear:.2f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

        if boca_aberta(pontos):
            if not boca_abrindo:
                boca_abrindo = True
                tempo_boca_aberta = time.time()
        else:
            boca_abrindo = False

        if ear <= LIMIAR_FECHAR:
            frames_fechado += 1
            if frames_fechado == 3:
                piscadas += 1
                arduino.write(b'G')
            elif frames_fechado > LIMIAR_COCHILO:
                arduino.write(b'R')

        elif ear > LIMIAR_ABRIR:
            frames_fechado = 0
            arduino.write(b'G')

        if boca_abrindo and (time.time() - tempo_boca_aberta) >= 3 and frames_fechado > LIMIAR_COCHILO:
            if not buzzer_ativado:
                arduino.write(b'R')
                arduino.write(b'B')
                buzzer_ativado = True

        if not boca_abrindo and ear > LIMIAR_ABRIR and buzzer_ativado:
            arduino.write(b'0')
            buzzer_ativado = False

        cv2.putText(frame, f"PISCADAS: {piscadas}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Detector de Fadiga e Alerta", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
arduino.close()
cv2.destroyAllWindows()