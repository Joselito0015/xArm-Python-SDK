def FuncionPosicion(self,Y):
    """
    FuncionPosicion
    """
    if Y == 1:
        self._vars['p_x'] = 0
        self._vars['p_z'] = 540
        print("Posiciones de Nivel 1 seteadas")
    elif Y == 2:
        self._vars['p_x'] = 0
        self._vars['p_z'] = 360
        print("Posiciones de Nivel 2 seteadas")
    elif Y == 3:
        self._vars['p_x'] = 0
        self._vars['p_z'] = 470
    elif Y == 4:
        self._vars['p_x'] = 0
        self._vars['p_z'] = 470
    elif Y == 5:
        self._vars['x'] = 0
        self._vars['p_z'] = 470
    else:
        self._vars['p_x'] = 207
        self._vars['p_z'] = 112
