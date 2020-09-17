import fltk

from OpenGL import GL
from PIL import Image

class MyGLWindow( fltk.Fl_Gl_Window ):
    
    def __init__( self, x, y, w, h, l = "" ):
        fltk.Fl_Gl_Window.__init__( self, x, y, w, h, l )
        
        self.imageFile = Image.open( "1.jpg" )
        self.imageObj  = self.imageFile.tobytes("raw", "RGBX", 0, -1)
        
        self.texid = None
        
    def draw( self ):
        # print( "MyGLWindow::draw()" )
        
        GL.glClearColor(0.0, 0.0, 0.0, 0.0)
        GL.glClearDepth(1.0) 
        GL.glClear( GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        
        
        # set camera view
        GL.glMatrixMode( GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glOrtho( -1, 1, -1, 1, -1, 1 )
        
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        
        
        
        imageW = self.imageFile.size[0]
        imageH = self.imageFile.size[1]
        
        if( self.texid == None ):
        
            self.texid = GL.glGenTextures( 1 )
            GL.glEnable( GL.GL_TEXTURE_2D )
            GL.glBindTexture( GL.GL_TEXTURE_2D, self.texid )
            GL.glPixelStorei( GL.GL_UNPACK_ALIGNMENT,1 )
            GL.glTexImage2D( GL.GL_TEXTURE_2D, 0, 3, imageW, imageH, 0, 
                             GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, self.imageObj )
        
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_REPEAT)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_REPEAT)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_NEAREST)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_NEAREST)
            # # GL.glTexEnvf(GL.GL_TEXTURE_ENV, GL.GL_TEXTURE_ENV_MODE, GL.GL_DECAL)
        
        print( self.texid )
        
        GL.glColor4f(1,1,1,1)
        GL.glBegin(GL.GL_QUADS)

        GL.glTexCoord2f(0.0, 0.0)
        GL.glVertex3f( -0.5, -0.5, 0 )
        
        GL.glTexCoord2f(1.0, 0.0)
        GL.glVertex3f( 0.5, -0.5, 0 )
        
        GL.glTexCoord2f(1.0, 1.0)
        GL.glVertex3f( 0.5, 0.5, 0 )
        
        GL.glTexCoord2f(0.0, 1.0)
        GL.glVertex3f( -0.5, 0.5, 0 )
        
        GL.glEnd()
        
        
if __name__ == "__main__":
    
    myGLWindow = MyGLWindow( 0, 0, 720, 480 )
    
    myGLWindow.show()
    
    while myGLWindow.shown():
        myGLWindow.redraw()
        fltk.Fl_check()
        
    
    fltk.Fl_run()