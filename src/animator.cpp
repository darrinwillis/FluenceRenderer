#include "animator.h"

#include <sstream>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <iomanip>

using namespace std;

namespace CMU462
{

   string PathTracer::name() {
      return "CMU 15-462 PathTracer";
   }

   string PathTracer::info() {
      return "";
   }

   void PathTracer::init() {
      text_drawer.init(use_hdpi);
      b_HUD = true;
      cursor_moving_element = false;
   }

    // The root of all drawing calls in this project.
    void PathTracer::render()
    {
        enter_2D_GL_draw_mode();
        // TODO TODO
        // Iterate over all light sources
            // For source sample n rays
                // Trace the ray to determine intersection
                // Draw line from source to destination
         
        // Draw the HUD.
        if(b_HUD)
        {
            drawHUD();
        }

        exit_2D_GL_draw_mode();
    }

    /*
   void PathTracer::render_frames()
   {
      // allocate frame memory
      uint32_t* frame_out = new uint32_t[width * height];

      glReadBuffer(GL_BACK);
      glDrawBuffer(GL_BACK);

      glPushAttrib( GL_VIEWPORT_BIT );
      glViewport( 0, 0, width, height );

      glMatrixMode( GL_PROJECTION );
      glPushMatrix();
      glLoadIdentity();
      glOrtho( 0, width, 0, height, 0, 1 ); // flip y

      glMatrixMode( GL_MODELVIEW );
      glPushMatrix();
      glLoadIdentity();
      glTranslatef( 0, 0, -1 );

      double time;
      size_t frame_count = 0;
      while( frame_count < frame_total )
      {
         renderer->clear( Color( .6, .6, .95, .2 ) );

         time = frame_count;

         // Update character state for the current time step
         for( vector<Character>::iterator character  = actors.begin();
               character != actors.end();
               character ++ )
         {
            character->update( time );
         }

         // Update characters
         for( vector<Character>::iterator character  = actors.begin();
               character != actors.end();
               character++ )
         {
            character->integrate( time, simulationTimestep );
         }

         // Draw each character in they order they appear in the "actors"
         // list.  Note that this ordering effectivly determines the layering/
         // occlusion of objects in the scene.
         for( vector<Character>::iterator character  = actors.begin();
               character != actors.end();
               character ++ )
         {
            character->draw( renderer );
         }

         // Read pixels
         glFlush();
         glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, frame_out);
         fprintf(stdout, "\rWriting frames: %lu/%lu", frame_count + 1, frame_total);
         fflush(stdout);

         // Write to image
         ostringstream filename;
         filename << "frame_";
         filename << std::setw(4) << std::setfill('0') << frame_count;
         filename << string(".png");
         lodepng::encode(filename.str(), (unsigned char*) frame_out, width, height);

         frame_count++;
      }

      std::cout << std::endl;

      glMatrixMode( GL_PROJECTION );
      glPopMatrix();

      glMatrixMode( GL_MODELVIEW );
      glPopMatrix();
      glPopAttrib();

   }
*/

   void PathTracer::resize( size_t width, size_t height ) {

      this->width  = width;
      this->height = height;

      text_drawer.resize(width, height);
   }

   void PathTracer::keyboard_event( int key, int event, unsigned char mods  )
   {
      // Handle arrow keys to move around timeline.
      if( event == EVENT_PRESS || event == EVENT_REPEAT )
      {
         switch( mods )
         {
            case MOD_ALT:
               break;
            case MOD_SHIFT:
               break;
            default:
               break;
         }
      }

      if(event == EVENT_PRESS)
      {
         switch(key)
         {
            case KEYBOARD_UP:
            case KEYBOARD_HOME:
               break;
            case KEYBOARD_DOWN:
            case KEYBOARD_END:
               break;
            case ' ':
               break;
            case 'd':
            case 'D':
                      b_HUD = !b_HUD;
                      break;

                      // Export a sequence of animation frames.
            case 's':
            case 'S':
                      break;

                      // Remove KeyFrames.
            case KEYBOARD_BACKSPACE:
            case KEYBOARD_DELETE:
                      break;

            case '[':
                      break;
            case ']':
                      break;
         }
      }

   }

   void PathTracer::cursor_event( float x, float y)
   {
      cursorPoint = Vector2D( x, y );
   }

   // FIXME : This code is the same as the helpful decomposition in p3.
   //         We should probably put this function in the standard library.
   //         I think that we should inherit mouse_pressed(), released() etc
   //         from the base class.
   void PathTracer::mouse_event(int key, int event, unsigned char mods)
   {
      switch(event)
      {
         case EVENT_PRESS:
            switch(key) {
               case MOUSE_LEFT:
                  mouse_pressed(LEFT);
                  break;
               case MOUSE_RIGHT:
                  mouse_pressed(RIGHT);
                  break;
               case MOUSE_MIDDLE:
                  mouse_pressed(MIDDLE);
                  break;
            }
            break;
         case EVENT_RELEASE:
            switch(key) {
               case MOUSE_LEFT:
                  mouse_released(LEFT);
                  break;
               case MOUSE_RIGHT:
                  mouse_released(RIGHT);
                  break;
               case MOUSE_MIDDLE:
                  mouse_released(MIDDLE);
                  break;
            }
            // Signal the end of any joint or character movement.
            cursor_moving_element = false;
            break;
      }
   }

   void PathTracer::mouse_pressed( e_mouse_button b )
   {
      switch( b )
      {
         case LEFT:
            break;
         case RIGHT:
            break;
         case MIDDLE:
            break;
      }
   }

   void PathTracer::mouse_released( e_mouse_button b )
   {
      switch( b )
      {
         case LEFT:
            break;
         case RIGHT:
            break;
         case MIDDLE:
            break;
      }
   }

   void PathTracer::scroll_event( float offset_x, float offset_y )
   {
      if (offset_x || offset_y)
      {
         // FIXME : Put Relevant scroll behaviors here.
      }
   }

   // ====== DRAWING ========

   void PathTracer::enter_2D_GL_draw_mode()
   {
      // FIXME: Double check these they might need to be screen space, instead of window space.
      int screen_w = width;
      int screen_h = height;

      glPushAttrib( GL_VIEWPORT_BIT );
      glViewport( 0, 0, screen_w, screen_h );

      glMatrixMode( GL_PROJECTION );
      glPushMatrix();
      glLoadIdentity();
      glOrtho( 0, screen_w, screen_h, 0, 0, 1 ); // Y flipped !

      glMatrixMode( GL_MODELVIEW );
      glPushMatrix();
      glLoadIdentity();
      glTranslatef( 0, 0, -1 );
      //  glDisable(GL_DEPTH_TEST);
   }

   void PathTracer::exit_2D_GL_draw_mode()
   {
      glMatrixMode( GL_PROJECTION );
      glPopMatrix();

      glMatrixMode( GL_MODELVIEW );
      glPopMatrix();
      glPopAttrib();
   }

   // -- TEXT Drawing for the HUD.

   void PathTracer::drawHUD()
   {
      text_drawer.clear();

      ostringstream m1, m2, m3, m4, m5, m6;

      m1 << "type: ";

      const size_t size = 12;
      const float x0 = use_hdpi ? width - 200 * 2 : width - 200;
      const float y0 = use_hdpi ? 32*2 : 32;
      const int inc  = use_hdpi ? 40 : 20;
      const int indent = use_hdpi ? 20 : 10;
      float y = y0 + inc - size;

      Color text_color = Color(1.0, 1.0, 1.0);

      drawString(x0,        y, "CHARACTER", size, text_color); y += inc;
      drawString(x0+indent, y, m4.str(),    size, text_color); y += inc;
      drawString(x0+indent, y, m5.str(),    size, text_color); y += inc;
      drawString(x0+indent, y, m6.str(),    size, text_color); y += inc;
      drawString(x0,        y, "JOINT",     size, text_color); y += inc;
      drawString(x0+indent, y, m1.str(),    size, text_color); y += inc;
      drawString(x0+indent, y, m2.str(),    size, text_color); y += inc;
      drawString(x0+indent, y, m3.str(),    size, text_color); y += inc;

      glColor4f(0.0, 0.0, 0.0, 0.8);

      text_drawer.render();
   }

   inline void PathTracer::drawString( float x, float y, string str, size_t size, Color c )
   {
      text_drawer.add_line( ( x*2/width)  - 1.0,
                            (-y*2/height) + 1.0,
                            str, size, c );
   }

} // namespace CMU462
