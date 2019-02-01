/****************************************************************
                       Copyright (c)
                    Microsoft Corporation
                    All Rights Reserved
               Licensed under the MIT License.
****************************************************************/

// Associated header
//
#include "k4aimuwindow.h"

// System headers
//
#include <utility>

// Library headers
//
#include "k4aimgui_all.h"

// Project headers
//
#include "k4aimudatagraph.h"
#include "k4aviewererrormanager.h"
#include "k4awindowsizehelpers.h"

using namespace k4aviewer;

namespace
{
constexpr float AccelMinRange = 5.0f;
constexpr float AccelMaxRange = 100.0f;
constexpr float AccelDefaultRange = 20.0f;
constexpr float AccelScaleFactor = 1.0f;

constexpr float GyroMinRange = 50.0f;
constexpr float GyroMaxRange = 1000.0f;
constexpr float GyroDefaultRange = 200.0f;
constexpr float GyroScaleFactor = 1.0f;
} // namespace

K4AImuWindow::K4AImuWindow(std::string &&title, std::shared_ptr<K4AImuSampleSource> sampleSource) :
    m_sampleSource(std::move(sampleSource)),
    m_title(std::move(title)),
    m_accelerometerGraph("Accelerometer",
                         "X",
                         "Y",
                         "Z",
                         "m/s/s",
                         AccelMinRange,
                         AccelMaxRange,
                         AccelDefaultRange,
                         AccelScaleFactor),
    m_gyroscopeGraph("Gyroscope",
                     " Roll",
                     "Pitch",
                     "  Yaw",
                     "Rad/s",
                     GyroMinRange,
                     GyroMaxRange,
                     GyroDefaultRange,
                     GyroScaleFactor)
{
}

void K4AImuWindow::Show(K4AWindowPlacementInfo placementInfo)
{
    if (!m_failed && m_sampleSource->IsFailed())
    {
        K4AViewerErrorManager::Instance().SetErrorStatus(m_title + ": sample source failed!");
        m_failed = true;
    }

    if (m_failed)
    {
        ImGui::Text("Sample source failed!");
        return;
    }

    const k4a_imu_sample_t sample = m_sampleSource->GetLastSample();
    if (sample.acc_timestamp_usec != m_lastAccTimestamp)
    {
        m_lastAccTimestamp = sample.acc_timestamp_usec;

        m_accelerometerGraph.AddSample(sample.acc_sample, sample.acc_timestamp_usec);
        m_gyroscopeGraph.AddSample(sample.gyro_sample, sample.gyro_timestamp_usec);
    }

    // Sizing math
    //
    ImGuiStyle &style = ImGui::GetStyle();
    ImVec2 graphSize = placementInfo.Size;
    graphSize.x -= 2 * style.WindowPadding.x;

    graphSize.y -= GetTitleBarHeight();       // Title bar
    graphSize.y -= 2 * style.WindowPadding.y; // Window internal padding

    graphSize.y -= ImGui::GetTextLineHeightWithSpacing(); // Sensor temp label

    constexpr int numSeparators = 2;
    constexpr int itemSpacingPerSeparator = 2;
    graphSize.y -= numSeparators * itemSpacingPerSeparator * style.ItemSpacing.y; // Separators

    constexpr int graphCount = 2;
    graphSize.y /= graphCount;

    // Actually draw the widgets
    //
    m_accelerometerGraph.Show(graphSize);

    ImGui::Separator();

    m_gyroscopeGraph.Show(graphSize);

    ImGui::Separator();
    ImGui::Text("Sensor temperature: %.2f C", double(sample.temperature));
}

const char *K4AImuWindow::GetTitle() const
{
    return m_title.c_str();
}
