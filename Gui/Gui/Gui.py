"""librerias para Vision artificial  """
import os
# Import required Libraries
import tkinter 
from tkinter import *
from PIL import Image, ImageTk, ImageGrab
import cv2
import sys
import numpy as np
import time
from threading import Timer
global matriz
b_cerrar=0
np.set_printoptions(threshold=np.inf)
# Capture from camera
cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture(cv2.CAP_V4L2)
#Verificamos que la webcam funciona
if not cap.isOpened():
    print("No se puede abrir la camara")
    sys.exit()


"""Librerias para definicion del robot y generacion de trayectoria '"""
from spatialmath.base import trotz, transl
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import roboticstoolbox as rtb 
from spatialmath import SE3
import pandas as pd
#import matplotlib.pyplot  as plt

"""librerias para el control de servo motores"""
import time
import sys
import signal
from PyMata.pymata import PyMata
SERVO_MOTOR1 = 2                                                   # servo attached to this pin
SERVO_MOTOR2 = 3
SERVO_MOTOR3= 5
SERVO_MOTOR4= 6
q1_ant=0
#Definición de los puertos para el motor paso a paso
num_steps=800 # default de numeros de pasos entre angulos(modificar a gusto)
pins=[8,9,10,11]
port= 'COM5'
board = PyMata(port)    
board.servo_config(SERVO_MOTOR1)
board.servo_config(SERVO_MOTOR2)
board.servo_config(SERVO_MOTOR3)
board.servo_config(SERVO_MOTOR4)     
board.stepper_config(num_steps,pins)# configuracion del stepper

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!!!!')
    if board is not None:
        board.reset()
    sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

def espera():
    q2=90  #eslabon1
    q3=90 #eslabon2
    q4=90    #eslabon3
    q5=170  #eslabon4
    T=SE3(0.26,0.400,0.57)
    #T = soldador.fkine([0, 0, 0, 0, 0,0])
    print(T)
    sol= soldador.ikine_LM(T)
    # # print(sol.q)
    # # time.sleep(0.5)  
    soldador.plot(sol.q)
    # qv=sol.q
    # #print(qv)
    # q2=qv[2]  #eslabon1
    # q3=qv[3] #eslabon2
    # q4=qv[4]    #eslabon3
    # q5=qv[5]  #eslabon4   
    # q22=round(92-(q2*(180/np.pi)))  #eslabon1
    # q33=round(10+(q3*(180/np.pi))) #eslabon2
    # q44=round(q4*(180/np.pi))+3  #eslabon3
    # q55=round(-(q5*(180/np.pi)))+4   #eslabon4
    # print(q22)
    # print(q33)
    # print(q44)
    # print(q55)
    # board.stepper_step(,-510) crear una funcion para volver al cero
    #time.sleep(1)
    board.analog_write(SERVO_MOTOR1, q2)
    #time.sleep(1)
    board.analog_write(SERVO_MOTOR2, q3)
    #time.sleep(1)
    board.analog_write(SERVO_MOTOR3, q4)
    #time.sleep(1)
    board.analog_write(SERVO_MOTOR4, q5)
    #time.sleep(1)
    

"""DEFINICION DEL ROBOT USANDO LA LIBRERIA ROBOTICTOOLS"""
%matplotlib Qt
class Soldador(DHRobot):

    def __init__(self):

        # deg = np.pi/180
        tool_offset = (00)*1e-3 # cambiar despues de hacer la herramienta

 #       flange = (80) * mm
        # d7 = (58.4)*mm

        # Denavit-Hartenberg parameters
        L = [
            
             RevoluteDH(                                    #primer Elemento
                  a=0,
                  d=0,
                  alpha=np.pi/2,
                  offset=0,
                  qlim=np.array([0.0,0.00]),
                  m=0,
                  I=[ 1,1,1,1,1,1,],
                  G=0,
                 
            ),
            PrismaticDH(                                                         #primer elemento
                a=0.0, 
                theta=0.0,
                alpha=0.0,
                qlim=np.array([0,1.2]),
                offset=0,
                m=1,
                I=[ 1,1,1,1,1,1,],
                G=1,
            ),
            RevoluteDH(                                                          #segundo Elemento
                a=0.255,
                d=0.0,
                alpha=0,
                offset=np.pi / 2,
                qlim=np.array([0,(np.pi/2)]),
                m=1,
                I=[ 1,1,1,1,1,1,],
                G=1,
            ),
            RevoluteDH(                                                         #tercer Elemento
                a=0.105,
                d=0,
                alpha=0,
                offset=-np.pi / 2,
                qlim=np.array([0, 2.8973]),
                m=1,
                I=[ 1,1,1,1,1,1,],
                G=1,
            ),
            RevoluteDH(                                                         #cuarto elemento
                a=0.195,
                d=0.0,
                alpha=0,
                offset=-np.pi / 2,
                qlim=np.array([0.1,np.pi/2]),
                m=1,
                I=[ 1,1,1,1,1,1,],
                G=1,
            ),
            RevoluteDH(                                                          #quinto elemento
                a=0.1,
                d=0,
                alpha=np.pi / 2,
                offset=np.pi/2 ,
                qlim=np.array([-np.pi/2,np.pi/2]),
                m=1,
                I=[ 1,1,1,1,1,1,],
                G=1,
            ),
        ]

        tool = transl(tool_offset, 0,0 ) 

        super().__init__(
            L,
            name="Soldador",
            manufacturer="HANS/LEPO/CARLOS",
            #meshdir="---",
            tool=tool,
        )
        
        self.qr = np.array([0, 0, 0, 0, 0,0])
        self.qz = np.zeros(6)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragrama nocover 

    soldador = Soldador()
    print(soldador)
    espera()
    





# FUNCIONES A IMPLEMENTAR =====================================================

def reduccion(frame):
    """
    Esta funcion reduce la escala de la imagen capturada
    """
    scale_percent = 70 # percent of original size
    width = int(frame.shape[1] * scale_percent / 100)
    height = int(frame.shape[0] * scale_percent / 100)
    dim = (width, height)
  
    # resize image
    resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
    return resized


# function for video streaming
def video_stream():
    """
    Esta funcion se encarga de realizae el streaming de lo que captura la camara
    para luego mostrarla en el GUI
    """
    ret, frame = cap.read()
    if not ret:
        print("No puede recibir el frame(se acabo el stream)...Saliendo")
        sys.exit()
    
    frame=reduccion(frame)
        
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    lmain.after(1, video_stream) 
    
    if b_cerrar==1:
        #Cuando se termina el proceso, se libera la captura
        cap.release()
        cv2.destroyAllWindows()
    



def capturar():
    """
    Esta funcion se encarga de realizar una captura del streaming, guarda la 
    captura en el archivo correspondiente, y luego actualiza la ultima
    captura realizada en el GUI
    
    """
    #Realizamos la captura de pantalla      
    _, frame = cap.read()
    
    frame=reduccion(frame)
    #Guardamos la captura en una imagen
    cv2.imwrite("captura.png",frame)
    
    #Actualizamos la ultima captrura de pantalla en el GUI
    img2=ImageTk.PhotoImage(Image.open("captura.png"))
    lmain2.configure(image=img2)
    lmain2.image=img2



def filtrar_color(img):
    """
    Esta funcion se encarga de filtrar por color la imagen que recibe, y 
    retorna la imagen filtrada
    """
    #Limites para el naranja(este es el color que planeamos usar)
    lower_c = np.array([10, 100, 20])
    upper_c = np.array([25, 255, 255])
    
    #Limites para el rojo(este es el que nos salio mejor hasta ahora)
    lower_c = np.array([160,100,50])
    upper_c = np.array([180,255,255])
    
    #Limites para el rojo(aca inicia mi tanteo)
    lower_c = np.array([140,70,50])
    upper_c = np.array([230,255,255])
    
    
    
    
    #Relizamos el filtrado de color en la imagen
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,lower_c,upper_c)
    result = cv2.bitwise_and(img,img, mask=mask)
    
    #Guardamos el resultado final en un png
    cv2.imwrite("procesado.png",result)
    print("Filtrado de color:\n")
   # print(result)
    return result



def erosionar(img):
    """
    Esta funcion se encarga de erosionar la imagen de entrada
    """
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
    #kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    erosion=cv2.erode(img,kernel,iterations=1)
    cv2.imwrite("erosionado.png",erosion)
    print("\nErosion:\n")
   # print(erosion)
   
    return erosion
    

def bitmap(img):
    ary = np.array(img)
    
    # Split the three channels
    r,g,b = np.split(ary,3,axis=2)
    r=r.reshape(-1)
    g=r.reshape(-1)
    b=r.reshape(-1)
    
    # Standard RGB to grayscale 
    bitmap = list(map(lambda x: 0.299*x[0]+0.587*x[1]+0.114*x[2],zip(r,g,b)))
    bitmap = np.array(bitmap).reshape([ary.shape[0], ary.shape[1]])
    bitmap = np.dot((bitmap > 30).astype(float),255)
    
    #Erosionamos el bitmap resultante
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 3))
    #kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    erosion=cv2.erode(bitmap,kernel,iterations=1)
    
    im = Image.fromarray(bitmap.astype(np.uint8))
    im.save('bitmap.bmp')
    print("\nResultado en bitmap:")
    #print(erosion/255)
    
    return im



def actualizar_imagenes():    
    img3=ImageTk.PhotoImage(Image.open("procesado.png"))
    lmain3.configure(image=img3)
    lmain3.image=img3
    
    img4=ImageTk.PhotoImage(Image.open("erosionado.png"))
    lmain4.configure(image=img4)
    lmain4.image=img4
    
    img5=ImageTk.PhotoImage(Image.open("bitmap.bmp"))
    lmain5.configure(image=img5)
    lmain5.image=img5
    
    img6=ImageTk.PhotoImage(Image.open("extra.png"))
    lmain6.configure(image=img6)
    lmain6.image=img6

def contornos(img):
    #METODO DE ENCONTRAR CONTORNOS
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cnts = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    #print(cnts)
    
    
    for c in cnts:
        cv2.drawContours(img, [c], -1, (255,255,255), thickness=1)
   
    
   
    file = open("contornos.txt", "w+")
 
    # Saving the 2D array in a text file
    content = str(cnts)
    file.write(content)
    file.close()
    
    cv2.imwrite("extra.png",img)
    
    return cnts


def dilatacion(img):
    # METODO DE DILATACION
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    dilate = cv2.dilate(img, kernel, iterations=3)
    cv2.imwrite("extra.png",dilate)

def procesar():
    img=cv2.imread("captura.png")
    if img is None:
            sys.exit("No se pudo leer la imagen")
    result=filtrar_color(img)
    erosionado=erosionar(result)
    mapa_bits=bitmap(result)
    m=contornos(result)
    #dilatacion(result)
    #Actualizamos la imagen procesada en el GUI
    actualizar_imagenes()  
    



def rehabilitar():
    """
    Esta funcion rehabilita los botones desabilitados por el proceso de soldadura
    """
    b1.config(state='normal',background='green',text="Captura")
    b2.config(state='normal',background='red',text="Cerrar")
    b3.config(state='normal',background='orange',text="Procesar")
    b4.config(state='normal',background='blue',text="Soldar")
    
    print("\nSoldadura Concretada")


#LECTURA Y PROCESO PARA EL MOVIMIENTO PARTE1%%%%%%%%%%%%%%%%%%%%%%%
def lectura_de_cuerpos(mm):
     cant_cuerpos=len(mm)  #cantidad de cuerpos detectados por el programa
     contador=0
     while contador!=cant_cuerpos:
         cuerpoX=mm[contador]# en este punto tengo un vector de n puntos agrupados en ventores  de la  forma [X,Y]
         cargaMtemporal(cuerpoX)
         espera()
         #time.sleep(5)
         contador=contador+1
         


# recibimos un cuerpo y creamos un espacio temporal de trabajo para ubicar los puntos 
#luego hacemos la interpolacion y creamos la recta que se ajusta a la curva
# a partir de ahí vamos moviendo el brazo hasta terminar con el cuerpo
def cargaMtemporal(cuerpoR):
    global q1_ant
    paso_vertical=1.0                        # relacion entre pixeles y longitud real 
    paso_horizontal=0.78125
    param=cuerpoR.shape
    longitudCuerpo=param[0]
    #print(longitudCuerpo)
                                        #vectores de posicion xyz, respecto al sistema de base
    vy=np.zeros(longitudCuerpo)   
    vz=np.zeros(longitudCuerpo)
    vx=0.27
    
    q1=np.zeros(longitudCuerpo)
    q2=np.zeros(longitudCuerpo)
    q3=np.zeros(longitudCuerpo)
    q4=np.zeros(longitudCuerpo)
    q5=np.zeros(longitudCuerpo)
    contador2=0
    
    while contador2<longitudCuerpo:
        vectorPix=cuerpoR[contador2]
        #print(vectorPix)
        vy[contador2]=((vectorPix[0][0]*paso_horizontal)/1000)+0.2     
        vz[contador2]=(((384-vectorPix[0][1])*paso_vertical)/1000)+0.12
        print(vx,vy[contador2],vz[contador2])
        T=SE3(vx,vy[contador2],vz[contador2])
        #print(T)
        sol= soldador.ikine_LM(T)
       # time.sleep(1)
        #soldador.plot(sol.q)
        qv=sol.q
        print(qv)
        q1[contador2]=(-qv[1])-0.2+0.03                          #paso a paso en metros
        q2[contador2]=round(92-(qv[2]*(180/np.pi)))  #eslabon1
        q3[contador2]=round(10+(qv[3]*(180/np.pi))) #eslabon2
        q4[contador2]=round(qv[4]*(180/np.pi))+3  #eslabon3
        q5[contador2]=round(-(qv[5]*(180/np.pi)))+6   #eslabon4
        # q2[contador2]=round(90-(qv[2]*(180/np.pi)))  #eslabon1
        # q3[contador2]=round(10+(qv[3]*(180/np.pi))) #eslabon2
        # q4[contador2]=round(qv[4]*(180/np.pi)+8)    #eslabon3
        # q5[contador2]=round((qv[5]*(180/np.pi)))    #eslabon4
        #print(q1[contador2])
        # print(q2[contador2])
        # print(q3[contador2])
        # print(q4[contador2])
        # print(q5[contador2])
        print(contador2)
        contador2=contador2+1
        # q22=round(92-(q2*(180/np.pi)))  #eslabon1
        # q33=round(10+(q3*(180/np.pi))) #eslabon2
        # q44=round(q4*(180/np.pi))+3  #eslabon3
        # q55=round(-(q5*(180/np.pi)))+4   #eslabon4
    q2= q2.astype(int)
    q3= q3.astype(int)
    q4= q4.astype(int)
    q5= q5.astype(int)
    
    contador2=0
    while contador2<longitudCuerpo:
        if contador2==0:
            q1_ant=0
            motores(q1[contador2],90,90,90,170)
            time.sleep(5)
        elif q2[contador2-1]!=q2[contador2]or q3[contador2-1]!=q3[contador2]or q4[contador2-1]!=q4[contador2]or q5[contador2-1]!=q5[contador2] :
            motores(q1[contador2],q2[contador2],q3[contador2],q4[contador2],q5[contador2])
            time.sleep(0.5)
        contador2=contador2+1
        
#      move the servo and stepper     
def motores(q1,q2,q3,q4,q5):
     global q1_ant
     #print(q1)
     qdif=q1- q1_ant #longitud en metros
     n_steps=round((qdif*2000)/0.021)                                       #5120 pasos equivalen a 51.32 cm
     print(n_steps)
     print(q2)
     print(q3)
     print(q4)
     print(q5)
     print('next' )
    # time.sleep(0.5)
     if (q2==90 and q3==90 and q4==90 and q5==170):
         contador3=0
         for i in range(0,n_steps,100): 
             board.stepper_step(50,100)
             contador3=contador3+100
             print (contador3)
             time.sleep(0.2)
     else:
         board.stepper_step(50,n_steps)
         time.sleep(1)
     # move the servo to 100 degrees
     board.analog_write(SERVO_MOTOR1,  q2)
     time.sleep(0.3)
     board.analog_write(SERVO_MOTOR2,  q3)
     time.sleep(0.3)
     board.analog_write(SERVO_MOTOR3,  q4)
     time.sleep(0.3)
     board.analog_write(SERVO_MOTOR4,  q5)
     time.sleep(0.3)

     # board.stepper_step(50,n_steps)
     # time.sleep(1)
     # board.analog_write(SERVO_MOTOR1, q2)
     # time.sleep(0.05)
     # board.analog_write(SERVO_MOTOR2, q3)
     # time.sleep(0.05)
     # board.analog_write(SERVO_MOTOR3, q4)
     # time.sleep(0.05)    
     # board.analog_write(SERVO_MOTOR4, q5)
     # time.sleep(1) 
     q1_ant= q1 
    
     
     

     
def soldar():
    """
    Aqui se cargan las funciones que realizan la soldadura
    """
    # Deshabilitamos los botones para evitar accidentes
    b1.config(state='disabled',background='gray',text="Deshabilitado")
    b2.config(state='disabled',background='gray',text="Deshabilitado")
    b3.config(state='disabled',background='gray',text="Deshabilitado")
    b4.config(state='disabled',background='gray',text="Soldando")
    # board.close()
    #b_cerrar=1
                                                                              # Iniciamos el proceso de soldadura
    
    print("\nProceso de soldadura")
    
    matriz = cv2.imread("extra.png",0)
  
    # print(matriz)
    # matriz=normalizar(matriz)
    # print(matriz)
    # print(matriz.shape)
    # escala=escalar(matriz)
    
    
    img=cv2.imread("captura.png")
    if img is None:
            sys.exit("No se pudo leer la imagen")
    
    result=filtrar_color(img)
    erosionado=erosionar(result)
    mapa_bits=bitmap(result)
    m=contornos(result)
    #dilatacion(result)
# Vamos a la funcion para generar la trayectoria de cada cuerpo=============
    lectura_de_cuerpos(m)  
    #print("La escala es "+str(escala)+" pixeles/cm")
    
    t = Timer(3, rehabilitar)  
    t.start()
  
 

   
def normalizar(a):
    """
    Normaliza una matriz a de nxm tal que todo valor >0 se rescribe como 1
    """

    #print(a)
    #print(a.shape)
    
    for i in range(len(a)):
        for j in range(len(a[i])):
            if(a[i][j])>0:
                a[i][j]=1
    #print(a)
    #print(a.shape)
    
    return a
    
  
    
  
def escalar(a):
    #Aca hay un proceso de medicion
    punto1=0
    punto2=0
    for i in range(len(a)):
        for j in range(len(a[i])):
            if(a[i][j])>0:
                if punto1>0 and j>punto1+10:
                    punto2=j
                else:
                    punto1=j
                    
        if punto1>0 and punto2>0:
            print("\nPuntos de Calibracion Encontrados\n")
            print("P1="+str(punto1)+"\n")
            print("P2="+str(punto2)+"\n")
            break
                
    longitud=punto2-punto1
    escala=longitud/5
    return int(escala)




def cerrar(b_cerrar):
    """
    Esta funcion se encarga de cerrar la ventana del GIU y de resetear el kernel
    """
    # close the interface down cleanly
   # board.close()
    b_cerrar=1
    
    clear = lambda: os.system('clear')
    clear()
    
    root.destroy()
    os._exit(00)
    

    
    


# GUI ======================================================
# Create an instance of TKinter Window or frame

root = Tk()
#root=Toplevel()
root.title("PROYECTO FINAL DE ROBOTICA I - BRAZO SOLDADOR")
root.geometry("1545x850")


texto1=Label(root,text="Vision de la Camara")
texto1.config(font= ("Gill Sans MT","10","bold") )
texto1.grid(row=0,column=0,columnspan=3)

texto2=Label(root,text="Captura de Camara")
texto2.config(font= ("Gill Sans MT","10","bold") )
texto2.grid(row=0,column=3,columnspan=3)

texto3=Label(root,text="Imagen Filtrada")
texto3.config(font= ("Gill Sans MT","10","bold") )
texto3.grid(row=0,column=6,columnspan=3)

texto4=Label(root,text="Imagen Erosionada")
texto4.config(font= ("Gill Sans MT","10","bold") )
texto4.grid(row=2,column=0,columnspan=3)

texto4=Label(root,text="Mapa de Bits")
texto4.config(font= ("Gill Sans MT","10","bold") )
texto4.grid(row=2,column=3,columnspan=3)

texto5=Label(root,text="Engrosado")
texto5.config(font= ("Gill Sans MT","10","bold") )
texto5.grid(row=2,column=6,columnspan=3)


# Ventana del streaming
app = Frame(root, bg="white")
app.grid(row=1,column=0,columnspan=3)
# Create a label in the frame
lmain = Label(app)
lmain.pack()


#Cargamos la imagen por primera vez
img1= ImageTk.PhotoImage(Image.open("captura.png"))
#Create a Label widget
lmain2= Label(root,image= img1)
lmain2.grid(row=1,column=3,columnspan=3)


#Cargamos cargamos la imagen con colores filtrados
img2= ImageTk.PhotoImage(Image.open("procesado.png"))
#Create a Label widget
lmain3= Label(root,image= img2)
lmain3.grid(row=1,column=6,columnspan=3)


#Cargamos cargamos la imagen erosionada
img3= ImageTk.PhotoImage(Image.open("erosionado.png"))
#Create a Label widget
lmain4= Label(root,image= img3)
lmain4.grid(row=3,column=0,columnspan=3)

#Cargamos cargamos la imagen mapeada
img4= ImageTk.PhotoImage(Image.open("bitmap.bmp"))
#Create a Label widget
lmain5= Label(root,image= img4)
lmain5.grid(row=3,column=3,columnspan=3)


#Cargamos cargamos la imagen mapeada
img5= ImageTk.PhotoImage(Image.open("extra.png"))
#Create a Label widget
lmain6= Label(root,image= img5)
lmain6.grid(row=3,column=6,columnspan=3)



# Botones del GUI
b1=Button(root,text="Captura",background="green",foreground="white",command=lambda:capturar())
b1.config(font= ("Arial","12","bold"))
b1.grid(row=4,column=0,columnspan=2)

b2=Button(root,text="Cerrar",background="red",foreground="white",command=lambda:cerrar(b_cerrar))
b2.config(font= ("Arial","12","bold"))
b2.grid(row=4,column=6,columnspan=3)

b3=Button(root,text="Procesar",background="orange",foreground="white",command=lambda:procesar())
b3.config(font= ("Arial","12","bold"))
b3.grid(row=4,column=3,columnspan=2)

b4=Button(root,text="Soldar",background="blue",foreground="white",command=lambda:soldar())
b4.config(font= ("Arial","12","bold"))
b4.grid(row=4,column=4,columnspan=2)

b5=Button(root,text="Medir",background="gray",foreground="white",command=lambda:soldar())
b5.config(font= ("Arial","12","bold"))
b5.grid(row=4,column=1,columnspan=2)



#Funciones que operan en todo momento
video_stream()


root.mainloop()