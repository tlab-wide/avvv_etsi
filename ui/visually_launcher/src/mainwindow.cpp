#include "mainwindow.hpp"
#include "./ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , rsu_slm_()
    , obu_slm_()
    , is_valid_rosbag_selected_(false)
    , is_valid_input_rosbag_folder_selected_(false)
    , is_valid_input_pcap_folder_selected_(false)
    , is_valid_output_folder_selected_(false)
    , is_valid_reporter_rosbag_selected_(false)
    , is_valid_reporter_output_selected_(false)
    , valid_pointcloud_(false)
    , valid_lanelet_(false)
{
    ui->setupUi(this);

    ui->AnalyserRosbagList->setModel(&rosbag_slm_);
    ui->AnalyserPcapList->setModel(&pcap_slm_);
    ui->VisualiserRsuList->setModel(&rsu_slm_);
    ui->VisualiserObuList->setModel(&obu_slm_);
    ui->NetworkSelectedRsu->setModel(&rsu_slm_);
    ui->NetworkSelectedObu->setModel(&obu_slm_);
    ui->ReporterAvailableGraphsList->setModel(&available_graphs_slm_);

    connect(&rosbag_process_, &QProcess::started, this, &MainWindow::rosbag_process_started);
    connect(&rosbag_process_, &QProcess::errorOccurred, this, &MainWindow::rosbag_process_error);
    connect(&rosbag_process_, &QProcess::finished, this, &MainWindow::rosbag_process_finished);

    connect(&main_app_process_, &QProcess::started, this, &MainWindow::main_app_process_started);
    connect(&main_app_process_, &QProcess::errorOccurred, this, &MainWindow::main_app_process_error);
    connect(&main_app_process_, &QProcess::finished, this, &MainWindow::main_app_process_finished);

    connect(&analyser_process_, &QProcess::started, this, &MainWindow::analyser_process_started);
    connect(&analyser_process_, &QProcess::errorOccurred, this, &MainWindow::analyser_process_error);
    connect(&analyser_process_, &QProcess::finished, this, &MainWindow::analyser_process_finished);

    connect(&reporter_process_, &QProcess::started, this, &MainWindow::reporter_process_started);
    connect(&reporter_process_, &QProcess::errorOccurred, this, &MainWindow::reporter_process_error);
    connect(&reporter_process_, &QProcess::finished, this, &MainWindow::reporter_process_finished);
}

MainWindow::~MainWindow()
{
    using namespace std::chrono_literals;

    if (main_app_process_.state() == QProcess::Running) {
        kill(main_app_process_.processId(), SIGINT);
        while (main_app_process_.state() == QProcess::Running)
            std::this_thread::sleep_for(100ms);
    }

    delete ui;
}


void MainWindow::on_AnalyserRosbagPath_textChanged(const QString &arg1)
{
    QString path{ ui->AnalyserRosbagPath->text() };
    QDir dir(path);

    QStringList name_filters;
    name_filters << "*.db3";

    QDirIterator dir_it(path, name_filters, QDir::Files, QDirIterator::Subdirectories);

    QStringList file_info_list;

    while (dir_it.hasNext())
        file_info_list << dir.relativeFilePath(dir_it.next());

    rosbag_slm_.setStringList(file_info_list);

    if (file_info_list.size())
        is_valid_input_rosbag_folder_selected_ = true;
    else
        is_valid_input_rosbag_folder_selected_ = false;
}


void MainWindow::on_AnalyserRosbagBrowse_clicked()
{
    QString path{ QFileDialog::getExistingDirectory(this, tr("Select a folder"), ".", QFileDialog::ShowDirsOnly) };
    ui->AnalyserRosbagPath->setText(path);
}


void MainWindow::on_AnalyserPcapPath_textChanged(const QString &arg1)
{
    QDir dir(ui->AnalyserPcapPath->text());

    QStringList name_filters;
    name_filters << "*.pcap";

    QStringList file_info_list{ dir.entryList(name_filters) };

    pcap_slm_.setStringList(file_info_list);

    if (file_info_list.size())
        is_valid_input_pcap_folder_selected_ = true;
    else
        is_valid_input_pcap_folder_selected_ = false;
}


void MainWindow::on_AnalyserPcapBrowse_clicked()
{
    QString path{ QFileDialog::getExistingDirectory(this, tr("Select a folder"), ".", QFileDialog::ShowDirsOnly) };
    ui->AnalyserPcapPath->setText(path);
}


void MainWindow::on_AnalyserOutputPath_textChanged(const QString &arg1)
{
    QDir dir(ui->AnalyserOutputPath->text());

    if (ui->AnalyserOutputPath->text().size() and dir.exists())
        is_valid_output_folder_selected_ = true;
    else
        is_valid_output_folder_selected_ = false;
}


void MainWindow::on_AnalyserOutputBrowse_clicked()
{
    QString path{ QFileDialog::getExistingDirectory(this, tr("Select a folder"), ".", QFileDialog::ShowDirsOnly) };
    ui->AnalyserOutputPath->setText(path);
}


void MainWindow::on_AnalyserAnalyse_clicked()
{
    try {
        if (ui->AnalyserAnalyse->text() == "Analyse") {
            if (not is_valid_input_rosbag_folder_selected_)
                throw std::runtime_error("Select a valid input ROSBAG folder");

            if (not is_valid_input_pcap_folder_selected_)
                throw std::runtime_error("Select a valid input PCAP folder");

            if (not is_valid_output_folder_selected_)
                throw std::runtime_error("Select a valid output folder");

            if (analyser_process_.state() == QProcess::Running)
                throw std::runtime_error("The application is already running");

            common::setAnalyseConf(
                        ui->AnalyserRosbagPath->text().toStdString()
                        , ui->AnalyserPcapPath->text().toStdString()
                        , ui->AnalyserOutputPath->text().toStdString());

            output_folder_directory_ = QString::fromStdString(ui->AnalyserOutputPath->text().toStdString());

            QStringList arguments;

            arguments << (std::string(std::getenv("AVVV_ETSI_HOME")) + std::string("analyser/main.py")).c_str() << "main";
            analyser_process_.start("python3", arguments);
        }
        else {
            if (analyser_process_.state() == QProcess::Running)
                analyser_process_.kill();
        }
    }
    catch (std::runtime_error& e) {
        ui->AnalyserError->setText(QString::fromStdString("Error: " + std::string(e.what())));
        ui->AnalyserSuccess->clear();
    }
}


void MainWindow::on_VisualiserBrowse_clicked()
{
    QString path{ QFileDialog::getOpenFileName(this, tr("Open ROSBAG files"), ".", tr("ROSBAG2 Files (*.db3)")) };
    ui->VisualiserRosbagPath->setText(path);
}


void MainWindow::on_VisualiserAnalyse_clicked()
{
    try {
        QFileInfo file_info{ ui->VisualiserRosbagPath->text() };
        if (not (file_info.exists() and file_info.isFile() and file_info.completeSuffix() == "db3"))
            throw std::runtime_error("Select a valid ROSBAG file");

        topics_ = common::getAllTopicsOfRosbag(ui->VisualiserRosbagPath->text().toStdString());
        QStringList rsu_ids;
        QStringList obu_ids;
        common::analyseRosbagTopics(topics_, rsu_ids, obu_ids);

        // Filling the ListViews and ComboBoxes
        rsu_slm_.setStringList(rsu_ids);
        obu_slm_.setStringList(obu_ids);

        ui->VisualiserRosbagError->clear();
        is_valid_rosbag_selected_ = true;
    }
    catch (std::runtime_error& e) {
        ui->VisualiserRosbagError->setText(QString::fromStdString("Error: " + std::string(e.what())));
    }
}


void MainWindow::on_VisualiserApply_clicked()
{
    try {
        if (not is_valid_rosbag_selected_)
            throw std::logic_error("Error: Select a valid ROSBAG file");
        QFileInfo file_info{ ui->VisualiserRosbagPath->text() };
        common::updateTopicsLaunch(topics_);
        common::updateGeneralTopicsRviz(topics_);
        ui->RosbagFileName->setText(file_info.completeBaseName() + "." + file_info.completeSuffix());
        ui->VisualiserApplyError->clear();
        ui->VisualiserApplySuccess->setText("Successfully applied settings");
    }
    catch (std::logic_error& e) {
        ui->VisualiserApplySuccess->clear();
        ui->VisualiserApplyError->setText(e.what());
    }
    catch (std::runtime_error&) {
        ui->VisualiserApplySuccess->clear();
        ui->VisualiserApplyError->setText("Error: Problem opening the launch file");
    }
    catch (std::exception&) {
        ui->VisualiserApplySuccess->clear();
        ui->VisualiserApplyError->setText("Error: You sourced the workspace, eh?");
    }
}


void MainWindow::on_NetworkDelay_currentIndexChanged(int index)
{
    ui->NetworkSuccess->clear();
    if (common::isSelectionValid(
                index,
                ui->NetworkJitter->currentIndex(),
                ui->NetworkRssi->currentIndex(),
                ui->NetworkPacketLoss->currentIndex()))
        ui->NetworkError->clear();
    else
        ui->NetworkError->setText("Remove duplicate values");
}


void MainWindow::on_NetworkJitter_currentIndexChanged(int index)
{
    ui->NetworkSuccess->clear();
    if (common::isSelectionValid(
                index,
                ui->NetworkDelay->currentIndex(),
                ui->NetworkRssi->currentIndex(),
                ui->NetworkPacketLoss->currentIndex()))
        ui->NetworkError->clear();
    else
        ui->NetworkError->setText("Remove duplicate values");
}


void MainWindow::on_NetworkRssi_currentIndexChanged(int index)
{
    ui->NetworkSuccess->clear();
    if (common::isSelectionValid(
                index,
                ui->NetworkDelay->currentIndex(),
                ui->NetworkJitter->currentIndex(),
                ui->NetworkPacketLoss->currentIndex()))
        ui->NetworkError->clear();
    else
        ui->NetworkError->setText("Remove duplicate values");

}


void MainWindow::on_NetworkPacketLoss_currentIndexChanged(int index)
{
    ui->NetworkSuccess->clear();
    if (common::isSelectionValid(
                index,
                ui->NetworkDelay->currentIndex(),
                ui->NetworkJitter->currentIndex(),
                ui->NetworkRssi->currentIndex()))
        ui->NetworkError->clear();
    else
        ui->NetworkError->setText("Remove duplicate values");

}


void MainWindow::on_HeatmapOfflineCheck_stateChanged(int arg1)
{
    ui->HeatmapOfflinePath->setEnabled(arg1);
    ui->HeatmapOfflineBrowse->setEnabled(arg1);
    ui->HeatmapOfflineAttribute->setEnabled(arg1);
    ui->label_11->setEnabled(arg1);
    if (not arg1)
        ui->HeatmapOfflinePath->clear();
}


void MainWindow::on_HeatmapOfflinePath_textChanged(const QString &arg1)
{
    QFileInfo file_info(arg1);
    if (file_info.exists()
            and file_info.isFile()
            and file_info.completeSuffix() == "csv") {
        ui->OfflineHeatmapName->setText(QString::fromStdString("Name: ") + file_info.completeBaseName());
        ui->OfflineHeatmapCreation->setText(QString::fromStdString("Creation Time: ") + file_info.birthTime().toString());
        ui->OfflineHeatmapSize->setText(QString::fromStdString("Size: ") + QString::number(file_info.size() / 1'000'000) + QString::fromStdString("MB"));
    }
    else {
        ui->OfflineHeatmapName->clear();
        ui->OfflineHeatmapCreation->clear();
        ui->OfflineHeatmapSize->clear();
    }
}


void MainWindow::on_HeatmapOfflineBrowse_clicked()
{
    QString path{ QFileDialog::getOpenFileName(this, tr("Open CSV files"), ".", tr("CSV files (*.csv)")) };
    ui->HeatmapOfflinePath->setText(path);
}


void MainWindow::on_NetworkApply_clicked()
{
    try {
        if (ui->NetworkError->text().length() == 0) { // If there are no errors in the network parameter selections

            if (ui->HeatmapOfflineCheck->isChecked() // Check the offline heatmap path selection
                    and ui->OfflineHeatmapName->text().length() == 0)
                throw std::runtime_error("Set the offline heatmap path properly");

            common::NetworkParameterRanges ranges{
                ui->NetworkDelayBest->text().toStdString(),
                ui->NetworkDelayWorst->text().toStdString(),
                ui->NetworkJitterBest->text().toStdString(),
                ui->NetworkJitterWorst->text().toStdString(),
                ui->NetworkRssiBest->text().toStdString(),
                ui->NetworkRssiWorst->text().toStdString(),
                ui->NetworkPacketLossBest->text().toStdString(),
                ui->NetworkPacketLossWorst->text().toStdString()
            };

            if (not common::validateNetworkRanges(ranges)) // Validate the network parameter ranges
                throw std::runtime_error("The network parameter ranges are invalid");

            common::updateNetworkGeneralLaunch(
                        ui->NetworkDelay->currentIndex(),
                        ui->NetworkJitter->currentIndex(),
                        ui->NetworkRssi->currentIndex(),
                        ui->NetworkPacketLoss->currentIndex(),
                        ranges);
        }

        common::updateOfflineHeatmapTopicsRviz(
                    ui->NetworkSelectedRsu->currentText().toStdString()
                    , ui->NetworkSelectedObu->currentText().toStdString()
                    , ui->HeatmapOfflineCheck->isChecked());

        common::updateOnlineHeatmapTopicsRviz(
                    topics_
                    , ui->HeatmapRealTimeCheck->isChecked());

        common::updateNetworkTargetedLaunch(
                    std::to_string(std::stod(ui->NetworkRsuObuConDist->text().toStdString()))
                    , ui->NetworkSelectedRsu->currentText().toStdString()
                    , ui->NetworkSelectedObu->currentText().toStdString()
                    , ui->HeatmapOfflinePath->text().toStdString()
                    , ui->HeatmapOfflineAttribute->currentIndex()
                    , ui->HeatmapRealTimeCheck->isChecked()
                    , ui->NetworkRealTimeGraphCheck->isChecked());

        ui->NetworkError->clear();
        ui->NetworkSuccess->setText("Successfully applied settings");
    }
    catch (std::invalid_argument&) {
        ui->NetworkSuccess->clear();
        ui->NetworkError->setText("Error: Invalid RSU-OBU connection range value");
    }
    catch (std::out_of_range&) {
        ui->NetworkSuccess->clear();
        ui->NetworkError->setText("Error: Invalid RSU-OBU connection range value");
    }
    catch (std::runtime_error& e) {
        ui->NetworkSuccess->clear();
        ui->NetworkError->setText(e.what());
    }
}


void MainWindow::on_PointcloudPath_textChanged(const QString &arg1)
{
    QFileInfo file_info(arg1);
    if (file_info.exists()
            and file_info.isFile()
            and file_info.completeSuffix() == "pcd") {
        ui->PointcloudName->setText(QString::fromStdString("Name: ") + file_info.completeBaseName());
        ui->PointcloudCreation->setText(QString::fromStdString("Creation Time: ") + file_info.birthTime().toString());
        ui->PointcloudSize->setText(QString::fromStdString("Size: ") + QString::number(file_info.size() / 1'000'000) + QString::fromStdString("MB"));
        valid_pointcloud_ = true;
    }
    else {
        valid_pointcloud_ = false;
        ui->PointcloudName->clear();
        ui->PointcloudCreation->clear();
        ui->PointcloudSize->clear();
    }
}


void MainWindow::on_PointcloudBrowse_clicked()
{
    QString path{ QFileDialog::getOpenFileName(this, tr("Open Pointcloud files"), ".", tr("Pointcloud Files (*.pcd)")) };
    ui->PointcloudPath->setText(path);
}


void MainWindow::on_LaneletPath_textChanged(const QString &arg1)
{
    QFileInfo file_info(arg1);
    if (file_info.exists()
            and file_info.isFile()
            and file_info.completeSuffix() == "osm") {
        ui->LaneletName->setText(QString::fromStdString("Name: ") + file_info.completeBaseName());
        ui->LaneletCreation->setText(QString::fromStdString("Creation Time: ") + file_info.birthTime().toString());
        ui->LaneletSize->setText(QString::fromStdString("Size: ") + QString::number(file_info.size() / 1'000'000) + QString::fromStdString("MB"));
        valid_lanelet_ = true;
    }
    else {
        valid_lanelet_ = false;
        ui->LaneletName->clear();
        ui->LaneletCreation->clear();
        ui->LaneletSize->clear();
    }
}


void MainWindow::on_LaneletBrowse_clicked()
{
    QString path{ QFileDialog::getOpenFileName(this, tr("Open Lanelet2 files"), ".", tr("Lanelet2 Files (*.osm)")) };
    ui->LaneletPath->setText(path);
}


void MainWindow::on_MapsApply_clicked()
{
    try {
        if (valid_pointcloud_ or ui->PointcloudPath->text() == "") {
            common::updatePointcloudPathLaunch(ui->PointcloudPath->text());
            common::updatePointcloudTopicsRviz(ui->PointcloudPath->text() != "");
        }
        else throw std::runtime_error("The pointcloud file does not exist or is not compatible");

        if (valid_lanelet_ or ui->LaneletPath->text() == "") {
            common::updateLaneletPathLaunch(ui->LaneletPath->text());
            common::updateLaneletTopicsRviz(ui->LaneletPath->text() != "");
        }
        else throw std::runtime_error("The lanelet file does not exist or is not compatible");

        common::updateMapOffsetLaunch(
                    ui->XOffset->cleanText()
                    , ui->YOffset->cleanText()
                    , ui->ZOffset->cleanText());

        common::updateFocalRviz(
                    ui->FocalX->cleanText()
                    , ui->FocalY->cleanText()
                    , ui->FocalZ->cleanText());

        ui->MapsError->clear();
        ui->MapsSuccess->setText("Successfully applied settings");
    }
    catch (std::runtime_error& e) {
        ui->MapsSuccess->clear();
        ui->MapsError->setText(e.what());
    }
}


void MainWindow::on_RosbagPlay_clicked()
{
    try {
        if (ui->RosbagPlay->text() == "Play") {
            if (rosbag_process_.state() == QProcess::Running)
                throw std::runtime_error("The ROSBAG is already running");

            QFileInfo file_info(ui->VisualiserRosbagPath->text());
            if (not (file_info.exists()
                    and file_info.isFile()
                    and file_info.completeSuffix() == "db3"))
                throw std::runtime_error("Error: Select a valid ROSBAG file");

            // To throw an std::invalid_argument exception if the play speed input field
            // was not a valid double and stop executing immediately
            std::stod(ui->RosbagPlaySpeed->text().toStdString());

            QStringList arguments;
            arguments << "bag" << "play" << ui->VisualiserRosbagPath->text() << "-r" << ui->RosbagPlaySpeed->text();
            if (ui->RosbagLoop->isChecked())
                arguments << "-l";
            rosbag_process_.start("ros2", arguments);
        }
        else {
            if (rosbag_process_.state() == QProcess::Running)
                rosbag_process_.kill();
        }
    }
    catch (std::runtime_error& e) {
        ui->RosbagPlaySuccess->clear();
        ui->RosbagPlayError->setText(e.what());
    }
    catch (std::invalid_argument&) {
        ui->RosbagPlaySuccess->clear();
        ui->RosbagPlayError->setText("Error: Enter a double value for 'playback speed'");
    }
    catch (std::out_of_range&) {
        ui->RosbagPlaySuccess->clear();
        ui->RosbagPlayError->setText("Error: Invalid range for 'playback speed'");
    }
}


void MainWindow::on_VisualiserLaunch_clicked()
{
    try {
        if (ui->VisualiserLaunch->text() == "Launch") {
            if (main_app_process_.state() == QProcess::Running)
                throw std::runtime_error("The application is already running");

            QStringList arguments;
            arguments << "launch" << "visually" << "visualiser.launch.py";
            main_app_process_.start("ros2", arguments);
        }
        else {
            if (main_app_process_.state() == QProcess::Running)
                kill(main_app_process_.processId(), SIGINT);
        }
    }
    catch (std::runtime_error& e) {
        ui->LaunchSuccess->clear();
        ui->LaunchError->setText(e.what());
    }
}


void MainWindow::on_ReporterDataPath_textChanged(const QString &arg1)
{
    QDir dir(arg1);

    QStringList name_filters;
    name_filters << "*.pickle";
    name_filters << "*.csv";

    QDirIterator dir_it(arg1, name_filters, QDir::Files, QDirIterator::Subdirectories);

    QStringList available_graphs;

    while (dir_it.hasNext())
        available_graphs << dir.relativeFilePath(dir_it.next());

    available_graphs_slm_.setStringList(available_graphs);
}


void MainWindow::on_ReporterOutputBrowse_clicked()
{
    QString path{ QFileDialog::getExistingDirectory(this, tr("Select a folder"), ".", QFileDialog::ShowDirsOnly) };
    ui->ReporterDataPath->setText(path);
}


void MainWindow::on_ReporterDisplay_clicked()
{
    try {
        if (not available_graphs_slm_.stringList().size())
            throw std::length_error("Nothing selected");

        QString dir_path{ ui->ReporterDataPath->text() };
        QStringList selected_graphs;

        for (const auto& index : ui->ReporterAvailableGraphsList->selectionModel()->selectedIndexes())
            selected_graphs << dir_path + "/" + index.data(Qt::DisplayRole).toString();

        if (not selected_graphs.size())
            throw std::length_error("Nothing selected");

        if (reporter_process_.state() == QProcess::Running)
            throw std::runtime_error("The application is already running");

        QStringList arguments;

        arguments << "analyser/main.py" << "plot";

        for (const auto& selected_graph : selected_graphs)
            arguments << selected_graph;

        reporter_process_.start("python3", arguments);
    }
    catch (std::length_error& e) {
        ui->ReporterSuccess->clear();
        ui->ReporterError->setText(e.what());
    }
    catch (std::runtime_error& e) {
        ui->ReporterSuccess->clear();
        ui->ReporterError->setText(e.what());
    }



}


void MainWindow::analyser_process_started()
{
    ui->AnalyserError->clear();
    ui->AnalyserSuccess->setText("Generating output...");
    ui->AnalyserAnalyse->setText("Abort");
}


void MainWindow::analyser_process_error()
{
    ui->AnalyserSuccess->clear();
    ui->AnalyserError->setText("Error: Could not start application.");
    QApplication::beep();
    ui->AnalyserAnalyse->setText("Analyse");
}


void MainWindow::analyser_process_finished()
{
    ui->AnalyserError->clear();
    ui->AnalyserSuccess->setText("Done.");
    QApplication::beep();

    ui->VisualiserRosbagPath->setText(output_folder_directory_.append("/outputs/rosbag2_avvv/rosbag2_avvv.db3"));
    ui->ReporterDataPath->setText(output_folder_directory_.append("/outputs"));
    ui->AnalyserAnalyse->setText("Analyse");
}


void MainWindow::rosbag_process_started()
{
    ui->RosbagPlayError->clear();
    ui->RosbagPlaySuccess->setText("Started playing the ROSBAG file");
    ui->RosbagPlay->setText("Stop");
}


void MainWindow::rosbag_process_error()
{
    ui->RosbagPlaySuccess->clear();
    ui->RosbagPlayError->setText("Error: Could not play ROSBAG, source the workspace");
    ui->RosbagPlay->setText("Play");
}


void MainWindow::rosbag_process_finished()
{
    ui->RosbagPlayError->clear();
    ui->RosbagPlaySuccess->setText("Finished playing the ROSBAG file");
    ui->RosbagPlay->setText("Play");
}


void MainWindow::main_app_process_started()
{
    ui->LaunchError->clear();
    ui->LaunchSuccess->setText("Application started.");
    ui->VisualiserLaunch->setText("Stop");
}


void MainWindow::main_app_process_error()
{
    ui->LaunchSuccess->clear();
    ui->LaunchError->setText("Error: Could not start application, source the workspace.");
    ui->VisualiserLaunch->setText("Launch");
}


void MainWindow::main_app_process_finished()
{
    ui->LaunchError->clear();
    ui->LaunchSuccess->setText("Application terminated.");
    ui->VisualiserLaunch->setText("Launch");
}


void MainWindow::reporter_process_started()
{
    ui->ReporterError->clear();
    ui->ReporterSuccess->setText("Displaying reports...");
}


void MainWindow::reporter_process_error()
{
    ui->ReporterSuccess->clear();
    ui->ReporterError->setText("Error: Could not start application.");
    QApplication::beep();
}


void MainWindow::reporter_process_finished()
{
    ui->ReporterError->clear();
    ui->ReporterSuccess->setText("Done.");
    QApplication::beep();
}


