#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <string>
#include <vector>
#include <csignal>
#include <thread>
#include <chrono>
#include <cstdlib> // For getting environment variables

#include <QMainWindow>
#include <QFileDialog>
#include <QFileInfo>
#include <QProcess>
#include <QStringList>
#include <QDir>
#include <QDirIterator>
#include <QApplication>
#include <QListWidget>
#include <QListWidgetItem>

#include "common.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_AnalyserRosbagPath_textChanged(const QString &arg1);

    void on_AnalyserRosbagBrowse_clicked();

    void on_AnalyserPcapPath_textChanged(const QString &arg1);

    void on_AnalyserPcapBrowse_clicked();

    void on_AnalyserOutputPath_textChanged(const QString &arg1);

    void on_AnalyserOutputBrowse_clicked();

    void on_AnalyserAnalyse_clicked();

    void on_VisualiserBrowse_clicked();

    void on_VisualiserAnalyse_clicked();

    void on_VisualiserApply_clicked();

    void on_NetworkDelay_currentIndexChanged(int index);

    void on_NetworkJitter_currentIndexChanged(int index);

    void on_NetworkRssi_currentIndexChanged(int index);

    void on_NetworkPacketLoss_currentIndexChanged(int index);

    void on_HeatmapOfflineCheck_stateChanged(int arg1);

    void on_HeatmapOfflinePath_textChanged(const QString &arg1);

    void on_HeatmapOfflineBrowse_clicked();

    void on_NetworkApply_clicked();

    void on_PointcloudPath_textChanged(const QString &arg1);

    void on_PointcloudBrowse_clicked();

    void on_LaneletPath_textChanged(const QString &arg1);

    void on_LaneletBrowse_clicked();

    void on_MapsApply_clicked();

    void on_RosbagPlay_clicked();

    void on_VisualiserLaunch_clicked();

    void on_ReporterDataPath_textChanged(const QString &arg1);

    void on_ReporterOutputBrowse_clicked();

    void on_ReporterDisplay_clicked();

    /**
     * @brief Analyser process start event handler
     */
    void analyser_process_started();

    /**
     * @brief Analyser process error event handler
     */
    void analyser_process_error();

    /**
     * @brief Analyser process finish event handler
     */
    void analyser_process_finished();

    /**
     * @brief ROSBAG process start event handler
     */
    void rosbag_process_started();

    /**
     * @brief ROSBAG process error event handler
     */
    void rosbag_process_error();

    /**
     * @brief ROSBAG process finish event handler
     */
    void rosbag_process_finished();

    /**
     * @brief Main application process start event handler
     */
    void main_app_process_started();

    /**
     * @brief Main application process error event handler
     */
    void main_app_process_error();

    /**
     * @brief Main application process finish event handler
     */
    void main_app_process_finished();

    /**
     * @brief Reporter process start event handler
     */
    void reporter_process_started();

    /**
     * @brief Reporter process start event handler
     */
    void reporter_process_error();

    /**
     * @brief Reporter process start event handler
     */
    void reporter_process_finished();

private:

    Ui::MainWindow *ui;

    // Contains the withdrawn topics from the ROSBAG
    std::vector<std::string> topics_;

    // Variables to maintain consistency in the
    bool is_valid_rosbag_selected_;
    bool is_valid_input_rosbag_folder_selected_;
    bool is_valid_input_pcap_folder_selected_;
    bool is_valid_output_folder_selected_;
    bool is_valid_reporter_rosbag_selected_;
    bool is_valid_reporter_output_selected_;
    bool valid_pointcloud_;
    bool valid_lanelet_;

    QString output_folder_directory_;

    QStringListModel rosbag_slm_;
    QStringListModel pcap_slm_;

    QStringListModel rsu_slm_;
    QStringListModel obu_slm_;

    QStringListModel available_graphs_slm_;

    // Processes that run external applications
    QProcess rosbag_process_;
    QProcess main_app_process_;
    QProcess analyser_process_;
    QProcess reporter_process_;

};
#endif // MAINWINDOW_HPP
