/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef GUI_H
#define	GUI_H

// Libraries
#include <sstream>
#include <iomanip>

#include <nanogui/opengl.h>
#include <nanogui/nanogui.h>
#include <nanogui/common.h>

#include "openglframework.h"
#include "Bone.h"
#include "VideoToBvhConverter.h"
#include "VideoController.h"

using namespace openglframework;
using namespace nanogui;

const double TIME_INTERVAL_DISPLAY_PROFILING_INFO = 1;

// Declarations
class TestbedApplication;

// Class Gui
class Gui {
    protected :

        enum LeftPane {SCENES, PHYSICS, RENDERING, PROFILING, TESTING};

        // -------------------- Constants -------------------- //
        int distanceBetweenWidgets = 10;

        // -------------------- Attributes -------------------- //

        // Pointer to the application
        TestbedApplication* mApp;

        // Screen
        Screen* mScreen;

        GLFWwindow* mWindow;

        static double mScrollX, mScrollY;

        // Simulation panel
        Widget* mSimulationPanel;

        // Settings Panel
        Widget* mSettingsPanel;
        Widget* mPhysicsPanel;
        Widget* mRenderingPanel;

        // Profiling panel
        Widget* mProfilingPanel;
        Label* mFPSLabel;
        Label* mFrameTimeLabel;
        Label* mTotalPhysicsTimeLabel;
        Label* mPhysicsStepTimeLabel;

        CheckBox* mCheckboxSleeping;
        CheckBox* mCheckboxGravity;
        TextBox* mTextboxTimeStep;
        TextBox* mTextboxVelocityIterations;
        TextBox* mTextboxPositionIterations;
        TextBox* mTextboxTimeSleep;
        TextBox* mTextboxSleepLinearVel;
        TextBox* mTextboxSleepAngularVel;

        ToolButton* mButtonPause;
        Widget* mPanelControls;

        std::vector<CheckBox*> mCheckboxesScenes;
        ComboBox* mComboBoxScenes;

        // Rotation panel
        Widget *mRotationPanel;
        TextBox *mRotateTextBox_x;
        Slider *mRotateSlider_x;

        TextBox *mRotateTextBox_y;
        Slider *mRotateSlider_y;

        TextBox *mRotateTextBox_z;
        Slider *mRotateSlider_z;

        bone::Bone *raycastedBone;

        std::vector<Label *> angleLabels;

        // Utils panel
        Widget *mUtilsPanel;
        std::string mBvhPath;
        std::string mVideoPath;
        TextBox *videoPath_textbox;
        TextBox *bvhPath_textbox;

        // -------------------- Image Viewer -------------------- //
        Window *bvhImageWindow;
        ImageView *bvhImageViewer;
        videoLoader::VideoController *pVideoController;

        // Analyze panel
        Widget *mAnalyzePanel;
        std::string mOpenposePath;

        /// True if the GUI is displayed
        bool mIsDisplayed;

        /// Current time (in seconds) from last profiling time display
        static double mTimeSinceLastProfilingDisplay;

        /// Cached Framerate
        static double mCachedFPS;

        /// Cached update time
        static double mCachedUpdateTime;

        // Cached total update physics time
        static double mCachedTotalPhysicsUpdateTime;

        // Cached update single physics step time
        static double mCachedPhysicsStepTime;

        // Current scene
        std::string mCurrentSceneName;

        // -------------------- Converter -------------------- //
        videoToBvhConverter::VideoToBvhConverter *pVideoToBvhConverter;

        // -------------------- Methods -------------------- //

        void createSimulationPanel();

        void createSettingsPanel();

        void createProfilingPanel();

        void createRotationPanel();

        void createUtilsPanel();

        void createAnalyzePanel();

        // Convert float value to string
        std::string floatToString(float value, int precision);

        static void resetScroll();

        void adjustRotationUtilsAnalyzePanel();
    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Gui(TestbedApplication* app);

        /// Destructor
        ~Gui();

        /// Initialize the GUI
        void init(GLFWwindow* window);

        /// Update the GUI
        void update();

        void drawAll();

        void draw();

        void drawTearDown();

        bool isFocus() const;

        static void setScroll(double scrollX, double scrollY);

        void createMessageDialog(const string &title, const string &message, MessageDialog::Type type, const std::function<void(int)>& callback);

        void createMessageDialog(const string &title, const string &message, MessageDialog::Type type);

        /// Update the GUI values with the engine settings from the current scene
        void resetWithValuesFromCurrentScene();

        // -------------------- Event -------------------- //
        void onChangeRaycastedTarget_bvhscene(bone::Bone *target);

        void onChangeBoneTransform_bvhscene(bone::Bone *target);

        void onCreateSkeleton_bvhscene();

        void onWindowResizeEvent(int width, int height);

        void onMouseMotionEvent(double x, double y);

        bool onScrollEvent(double x, double y);

        void onMouseButtonEvent(int button, int action, int modifiers);

        bool onKeyboardEvent(int key, int scancode, int action, int modifiers);

        bool onCharacterEvent(unsigned int codepoint);

        std::string onOpenFileButtonPressed(const vector<pair<string, string>> &valid, bool save);

        void onMotionNext();

        void onForearmStrokeAnalyzeDone();

        // -------------------- Getter & Setter -------------------- //
        bool getIsDisplayed() const;

        void setIsDisplayed(bool isDisplayed);
};

inline void Gui::resetScroll() {
    mScrollX = 0.0;
    mScrollY = 0.0;
}

inline void Gui::setScroll(double scrollX, double scrollY) {
    mScrollX = scrollX;
    mScrollY = scrollY;
}

inline std::string Gui::floatToString(float value, int precision) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

inline bool Gui::getIsDisplayed() const {
    return mIsDisplayed;
}

inline void Gui::setIsDisplayed(bool isDisplayed) {
    mIsDisplayed = isDisplayed;
}

#endif
