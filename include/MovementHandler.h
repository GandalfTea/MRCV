
#include <pangolin/gl/gl.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>

#include <Eigen/Dense>

namespace pangolin {
	
	struct MovementHandler : Handler3D {
		MovementHandler( OpenGlRenderState& cam_state, 
										 AxisDirection enforce_up = AxisNone,
										 float trans_scale = 0.01f,
										 float zoom_fraction = PANGO_DFLT_HANDLER3D_ZF)
						: Handler3D(cam_state, enforce_up, trans_scale, zoom_fraction) {};


	void MouseMotion( View& display, int x, int y, int button_state ) {
		const double rf = 0.01;
		const float delta[2] = { (float)x - last_pos[0], (float)y - last_pos[1] };
		const float mag = delta[0] * delta[0] + delta[1] * delta[1];

		if( (button_state & KeyModifierCtrl) && (button_state & KeyModifierShift)) {
			double T_nc[3*4];
			LieSetIdentity(T_nc);

			GetPosNormal(display, x, y, p, Pw, Pc, n, last_z);
			if( ValidWinDepth( p[2] )) {
				last_z = p[2];
				std::copy(Pc, Pc+3, rot_center);
			}

			funcKeyState = button_state;
		}

		else {
			funcKeyState=0;
		}

		if( mag < 50.0f * 50.0f ) {
			OpenGlMatrix& mv = cam_state->GetModelViewMatrix();
			const double* up = AxisDirectionVector[ enforce_up ];
			double T_nc[3*4];
			bool rotation_changed = false;

			if( button_state == MouseButtonMiddle ) {
				// Middle Drag: Rotate around view

				double aboutx = -rf * delta[1];
				double abouty = rf * delta[0];
				OpenGlMatrix& pm = cam_state->GetProjectionMatrix();
				abouty *= -pm.m[2 * 4 + 3];

				Rotation<>(T_nc, aboutx, abouty, (double)0.0);

			} else if ( button_state == MouseButtonLeft ) {
				// Left Drag: in plane translate
				if( ValidWinDepth(last_z) ) {
					double np[3];
					PixelUnproject(display, x, y, last_z, np);
					const double t[] = { np[0] - rot_center[0], np[1] - rot_center[1], 0};
					LieSetTranslation<>(T_nc, t);
					std::copy(np, np+3, rot_center);
				} else {
					const double t[] = { -10*delta[0]*tf, 10*delta[1]*tf, 0};
					LieSetTranslation<>(T_nc, t);
				}	

			} else if ( button_state == (MouseButtonLeft | MouseButtonRight) ) {
				// Left and Right Drag : in plane rotate about object
				
				double T_2c[3*4];
				Rotation<>(T_2c, (double)0.0, (double)0.0, delta[0]*rf);
				double mrotc[3];
				MatMul<3,1>(mrotc, rot_center, (double)-1.0);
				LieApplySO3<>(T_2c + (3*3), T_2c, mrotc);
				double T_n2[3*4];
				LieSetIdentity<>(T_n2);
				LieSetTranslation<>(T_n2, rot_center);
				LieMulSE3(T_nc, T_n2, T_2c);
				rotation_changed = true;

			} else if ( button_state == MouseButtonRight ) {
				double aboutx = -rf * delta[1];
				double abouty = -rf * delta[0];

				if( cam_state->GetProjectionMatrix().m[2 * 4 + 3] <= 0) {
					abouty *= -1;
				}

				if( enforce_up ) {
					// Special case if view direction is parallel to up vector
					const double updotz = mv.m[2] * up[0] + mv.m[6] * up[1] + mv.m[10] * up[2];
					if( updotz > 0.98) aboutx = std::min(aboutx, (double)0.0);
					if( updotz <-0.98) aboutx = std::max(aboutx, (double)0.0);
					abouty *= (1-0.6*fabs(updotz));
				}

				// Right Drag
				double T_2c[3*4];
				Rotation<>(T_2c, aboutx, abouty, (double)0.0);
				double mrotc[3];
				MatMul<3,1>(mrotc, rot_center, (double)-1.0);
				LieApplySO3<>(T_2c + (3*3), T_2c, mrotc);
				double T_n2[3*4];
				LieSetIdentity<>(T_n2);
				LieSetTranslation<>(T_n2, rot_center);
				LieMulSE3(T_nc, T_n2, T_2c);
				rotation_changed = true;

				//std::cout << cam_state->GetModelViewMatrix() << std::endl;

			}

			LieMul4x4bySE3<>(mv.m, T_nc, mv.m);

			if( enforce_up != AxisNone && rotation_changed) {
				EnforceUpT_cw(mv.m, up);
			}
		}
		last_pos[0] = (float)x;
		last_pos[1] = (float)y;
	}
};


} // namespace

