class Brazo():
    def __init__(self):
        self.nombre = "Brazo"
        self.posicionNeutra()
            
    def avanzarAlPallet(self):
        """Funcion que se encarga de avanzar el brazo hasta el pallet"""
        print("Moviendo a: ")
        return True
    
    def abrirPinza(self):
        """Funcion que se encarga de abrir la pinza"""
        print("Abriendo pinza")
        return True
    
    def cerrarPinza(self):
        """Funcion que se encarga de cerrar la pinza"""
        print("Cerrando pinza")
        return True
    
    def subirPinza(self):
        """Funcion que se encarga de subir la pinza 5 cm"""
        print("Subiendo pinza")
        return True

    def posicionNeutra(self, Almacen):
        """Funcion que se encarga de mover el brazo a la posicion neutra.
        - sube
        - sale *Depende del almacen en el que se encuentre
        - gira
        - pos Neutra
         """
        print("Moviendo a posición neutra")
        return True
    
    def nivelarAltura(self, y):
        """Funcion que se encarga de nivelar la altura del brazo según la altura ingresada por el Master"""
        print("Nivelando altura", y)
        return True

    def girarBrazo(self, Almacen):
        """Funcion que se encarga de girar el brazo al almacen indicado +-15"""
        print("Girando brazo al almacen", Almacen)
        return True
    
    def AlmacenarPallet(self, Almacen):
        """Funcion que se encarga de almacenar el pallet en el almacen indicado """
        print("Almacenando pallet en almacen", Almacen)
        return True