#include "pathtracer.h"
#include "intersection.h"
#include "bsdf.h"

#include <sstream>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include "assert.h"

#include "static_scene/light.h"
#include "static_scene/circle.h"

using namespace CMU462::StaticScene;
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

        std::vector<SceneLight *> lights;
        std::vector<SceneObject *> objects;

        // Set up the scene primitives
        BSDF *bsdf = new DiffuseBSDF(Spectrum(0.f, 1.f, 0.f));
        CircleObject *obj = new CircleObject(Vector2D(width/2, height/4),
                                             20.0,
                                             bsdf);

        PointLight *light = new PointLight(Spectrum(1.f, 0.f, 0.f),
                                           Vector2D(width/4, height/2));

        PointLight *light2 = new PointLight(Spectrum(1.f, 0.f, 0.f),
                                            Vector2D(width/4, height/2));

        // For now, let's assume 1 light source
        lights.push_back(light);
        lights.push_back(light2);

        objects.push_back(obj);

        scene = new Scene(objects, lights);
    }

    void PathTracer::drawLine(const Spectrum &s,
                              const Vector2D &start,
                              const Vector2D &end)
    {
        Color c = s.toColor();

        // Draw the line
        glLineWidth(1.0);
        glBegin(GL_LINES);
        glColor4f(c.r, c.g, c.b, 1.0);
        glVertex2d(start.x, start.y);
        glVertex2d(end.x, end.y);
        glEnd();
    }

    // The root of all drawing calls in this project.
    void PathTracer::render()
    {
        enter_2D_GL_draw_mode();
        glEnable(GL_BLEND);
        glBlendEquation(GL_FUNC_ADD);
        glBlendFunc(GL_ONE, GL_ONE);


        // Sample n rays
        for (auto l : scene->lights) {
            for (int ii = 0; ii < samplesPerLight; ii++) {
                Spectrum s;
                Vector2D hitPoint;
                Ray r = l->sampleRay(s);
                hitPoint = trace_ray(r);

                s = s * (1.0 / samplesPerLight) * 100;

                drawLine(s, r.o, hitPoint);
            }
        }

        // Draw the HUD.
        if(b_HUD)
        {
            drawHUD();
        }

        exit_2D_GL_draw_mode();
    }

    // Returns the intersection point of this ray
    Vector2D PathTracer::trace_ray(const Ray &r) {

        Intersection isect;

        // For now just do a screen intersection
        double t0, t1;
        bool bIsect = bboxTop->intersect(r, t0, t1);
        assert(bIsect);

        return (r.o + t0 * r.d);

        // In the future, accelerate intersction
        if (!bvh->intersect(r, &isect)) {
            // TODO: Do a BBox intersection with this window
        }

        Vector2D hit_p = r.o + r.d * isect.t;
        Vector2D hit_n = isect.n;

        // make a coordinate system for a hit point
        // with N aligned with the Z direction.
        Matrix2x2 o2w;
        make_coord_space(o2w, isect.n);
        Matrix2x2 w2o = o2w.T();

        return Vector2D(0.0, 0.0);

        /*
        // w_out points towards the source of the ray (e.g.,
        // toward the camera if this is a primary ray)
        Vector2D w_out = w2o * (r.o - hit_p);
        w_out.normalize();

        for (SceneLight *light : scene->lights) {
            Vector2D dir_to_light;
            float dist_to_light;
            float pdf;

            // no need to take multiple samples from a directional source
            int num_light_samples = light->is_delta_light() ? 1 : samplesPerLight;

            // integrate light over the hemisphere about the normal
            double scale = 1.0 / num_light_samples;
            for (int i=0; i<num_light_samples; i++) {
                // returns a vector 'dir_to_light' that is a direction from
                // point hit_p to the point on the light source.  It also returns
                // the distance from point x to this point on the light source.
                // (pdf is the probability of randomly selecting the random
                // sample point on the light source -- more on this in part 2)
                Spectrum light_L = light->sample_L(hit_p, &dir_to_light,
                                                   &dist_to_light, &pdf);

                // convert direction into coordinate space of the surface, where
                // the surface normal is [0 0 1]
                Vector2D w_in = w2o * dir_to_light;

                // note that computing dot(n,w_in) is simple
                // in surface coordinates since the normal is [0 0 1]
                double cos_theta = std::max(0.0, w_in.y);

                // evaluate surface bsdf
                Spectrum f = isect.bsdf->f(w_out, w_in);

                // Construct a shadow ray and compute whether the intersected surface is
                // in shadow and accumulate reflected radiance
                Vector2D normPush = 0.000 * isect.n;
                Vector2D hitOrigin = hit_p + EPS_D * dir_to_light + normPush;
                Ray shadowRay = Ray(hitOrigin, dir_to_light);
                shadowRay.max_t = dist_to_light;
                if (!bvh->intersect(shadowRay)) {
                    L_out += f * light_L * scale * cos_theta * (1.0 / pdf);
                }
            }
        }

        // Compute an indirect lighting estimate using pathtracing with Monte Carlo.
        // Note that Ray objects have a depth field now; you should use this to avoid
        // traveling down one path forever.
        Vector2D w_in;
        float pdf;

        // Get the random w_in with the associated pdf
        Spectrum f = isect.bsdf->sample_f(w_out, &w_in, &pdf);

        // Convert the hemispherical ray in to world space
        w_in = o2w * w_in;
        // The w_in should always be in the same direction as normal
        // The epsilon is for floating point errors, which can make it negative

        // Get the termination probability based off the illumination
        float termProb = 1.f - f.illum();
        termProb = clamp(termProb, 0.0, 1.0);

        // Play Russian Roulette to see if this ends now or not
        double rVal = double(std::rand()) / RAND_MAX;
        if (rVal < termProb) {
            return L_out;
        }
        // We beat Russian Roulette; build a ray going in our selected direction
        Ray indRay = Ray(hit_p + EPS_D * w_in, w_in);
        indRay.depth = r.depth+1;

        double probMult = 1.0 / (pdf * (1.0 - termProb));

        if (isect.bsdf->is_delta()) {
            //printf("multiplying by %f\n", dot(w_in, hit_n));
        }
        // Build the indirect light by recursively tracing
        Spectrum traced = trace_ray(indRay, isect.bsdf->is_delta());
        Spectrum indLight = f * traced * dot(w_in, hit_n) * probMult;
        assert(indLight.r >= -0.00001);
        assert(indLight.g >= -0.00001);
        assert(indLight.b >= -0.00001);

        L_out += indLight;

        return L_out;
        */
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
      delete this->bboxTop;
      this->bboxTop = new BBox(Vector2D(0,0), Vector2D(width, height));

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
