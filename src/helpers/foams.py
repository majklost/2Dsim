class Foam:
    """
    https://graphics.stanford.edu/~mdfisher/cloth.html
    https://www.scss.tcd.ie/michael.manzke/CS7057/cs7057-1516-14-MassSpringSystems-mm.pdf
    https://graphics.stanford.edu/courses/cs468-02-winter/Papers/Rigidcloth.pdf
    """
    def __init__(self,x,y,w,h,row_mass_num,col_mass_num, linearParams, thickness=2):
        self.segments = []
        self.segments_shapes = []
        self.structuralSprings = [] #horizontal and vertical
        self.shearSprings = [] #diagonal one
        self.joints = []
        self.row_mass_num = row_mass_num
        self.col_mass_num = col_mass_num
        self.linearParams = linearParams
        self.thickness = thickness



    class SpringParams:
        def __init__(self, stiffness, damping):
            self.stiffness = stiffness
            self.damping = damping