#include "mainwindow.hpp"
#include "./ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , rsu_slm_()
    , obu_slm_()
    , is_valid_rosbag_selected_(false)
    , is_valid_input_rosbag_folder_selected_(false)
    , is_valid_input_csv_folder_selected_(false)
    , is_valid_output_folder_selected_(false)
    , is_valid_reporter_rosbag_selected_(false)
    , is_valid_reporter_output_selected_(false)
    , valid_pointcloud_(false)
    , valid_lanelet_(false)
{
    ui->setupUi(this);

    ui->AnalyserRosbagList->setModel(&rosbag_slm_);
    ui->AnalyserCsvList->setModel(&csv_slm_);
    ui->CloudList->setModel(&clouds_slm_);
    ui->TrafficSignalList->setModel(&traffic_signals_slm_);
    ui->RsuList->setModel(&rsus_slm_);
    ui->ObuList->setModel(&obus_slm_);
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
    
    connect(&preprocessor_process_, &QProcess::started, this, &MainWindow::preprocessor_process_started);
    connect(&preprocessor_process_, &QProcess::errorOccurred, this, &MainWindow::preprocessor_process_error);
    connect(&preprocessor_process_, &QProcess::finished, this, &MainWindow::preprocessor_process_finished);
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


void MainWindow::on_AnalyserCsvPath_textChanged(const QString &arg1)
{
    QDir dir(ui->AnalyserCsvPath->text());

    QStringList name_filters;
    name_filters << "*.csv";

    QStringList file_info_list{ dir.entryList(name_filters) };

    csv_slm_.setStringList(file_info_list);

    if (file_info_list.size())
        is_valid_input_csv_folder_selected_ = true;
    else
        is_valid_input_csv_folder_selected_ = false;
}


void MainWindow::on_AnalyserCsvBrowse_clicked()
{
    QString path{ QFileDialog::getExistingDirectory(this, tr("Select a folder"), ".", QFileDialog::ShowDirsOnly) };
    ui->AnalyserCsvPath->setText(path);
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

            if (not is_valid_input_csv_folder_selected_)
                throw std::runtime_error("Select a valid input CSV folder");

            if (not is_valid_output_folder_selected_)
                throw std::runtime_error("Select a valid output folder");

            if (analyser_process_.state() == QProcess::Running)
                throw std::runtime_error("The application is already running");

            common::setAnalyseConf(
                        ui->AnalyserRosbagPath->text().toStdString()
                        , ui->AnalyserCsvPath->text().toStdString()
                        , ui->AnalyserOutputPath->text().toStdString());

            QStringList arguments;

            arguments << (std::string(std::getenv("AVVV_DM_HOME")) + std::string("analyser/main.py")).c_str() << "main";
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


void MainWindow::on_VisualiserRosbagBrowse_clicked()
{
    QString path{ QFileDialog::getOpenFileName(this, tr("Open ROSBAG files"), ".", tr("ROSBAG2 Files (*.db3)")) };
    ui->VisualiserRosbagPath->setText(path);
}


void MainWindow::on_VisualiserRosbagAnalyse_clicked()
{
    try {
        QFileInfo file_info{ ui->VisualiserRosbagPath->text() };
        if (not (file_info.exists() and file_info.isFile() and file_info.completeSuffix() == "db3"))
            throw std::runtime_error("Select a valid ROSBAG file");

        topics_ = common::getAllTopicsOfRosbag(ui->VisualiserRosbagPath->text().toStdString());

        ui->VisualiserRosbagError->setText("Successfully withdrew bag file topics.");
        is_valid_rosbag_selected_ = true;
    }
    catch (std::runtime_error& e) {
        ui->VisualiserRosbagError->setText(QString::fromStdString("Error: " + std::string(e.what())));
    }
}


void MainWindow::on_CloudAdd_clicked()
{
    if (not ui->CloudId->text().size())
        return;

    auto cloud_list{ clouds_slm_.stringList() };
    cloud_list << ui->CloudId->text();
    clouds_slm_.setStringList(cloud_list);
}


void MainWindow::on_CloudClear_clicked()
{
    auto selection_model{ ui->CloudList->selectionModel() };

    QStringList remaining;

    for (int idx{}; idx < clouds_slm_.stringList().size(); ++idx)
        if (not selection_model->isRowSelected(idx))
            remaining << clouds_slm_.stringList().at(idx);

    clouds_slm_.setStringList(remaining);
}


void MainWindow::on_TrafficAdd_clicked()
{
    if (not ui->TrafficSignalId->text().size())
        return;

    auto traffic_signal_list{ traffic_signals_slm_.stringList() };
    traffic_signal_list << ui->TrafficSignalId->text();
    traffic_signals_slm_.setStringList(traffic_signal_list);
}


void MainWindow::on_TrafficClear_clicked()
{
    auto selection_model{ ui->TrafficSignalList->selectionModel() };

    QStringList remaining;

    for (int idx{}; idx < traffic_signals_slm_.stringList().size(); ++idx)
        if (not selection_model->isRowSelected(idx))
            remaining << traffic_signals_slm_.stringList().at(idx);

    traffic_signals_slm_.setStringList(remaining);
}


void MainWindow::on_RsuAdd_clicked()
{
    if (not ui->RsuId->text().size())
        return;

    auto rsu_list{ rsus_slm_.stringList() };
    rsu_list << ui->RsuId->text();
    rsus_slm_.setStringList(rsu_list);
}


void MainWindow::on_RsuClear_clicked()
{
    auto selection_model{ ui->RsuList->selectionModel() };

    QStringList remaining;

    for (int idx{}; idx < rsus_slm_.stringList().size(); ++idx)
        if (not selection_model->isRowSelected(idx))
            remaining << rsus_slm_.stringList().at(idx);

    rsus_slm_.setStringList(remaining);
}


void MainWindow::on_ObuAdd_clicked()
{
    if (not ui->ObuId->text().size())
        return;

    auto obu_list{ obus_slm_.stringList() };
    obu_list << ui->ObuId->text();
    obus_slm_.setStringList(obu_list);
}


void MainWindow::on_ObuClear_clicked()
{
    auto selection_model{ ui->ObuList->selectionModel() };

    QStringList remaining;

    for (int idx{}; idx < obus_slm_.stringList().size(); ++idx)
        if (not selection_model->isRowSelected(idx))
            remaining << obus_slm_.stringList().at(idx);

    obus_slm_.setStringList(remaining);
}


void MainWindow::on_VisualiserApply_clicked()
{
    try {
        if (not is_valid_rosbag_selected_)
            throw std::logic_error("Error: Select a valid ROSBAG file");
        QFileInfo file_info{ ui->VisualiserRosbagPath->text() };

        auto rsu_ids{ common::getRsuIds(rsus_slm_.stringList()) };
        auto obu_ids{ common::getObuIds(obus_slm_.stringList()) };
        auto cloud_ids{ common::getCloudIds(clouds_slm_.stringList()) };

        common::refineTopics(
                    rsu_ids,
                    obu_ids,
                    topics_,
                    rsu_object_topics_,
                    rsu_freespace_topics_,
                    rsu_signal_topics_,
                    obu_object_topics_,
                    obu_freespace_topics_,
                    obu_signal_topics_,
                    obu_tf_topics_,
                    link_topics_);

        common::updateLaunchConfigurations(
                    rsus_slm_.stringList(),
                    obus_slm_.stringList(),
                    clouds_slm_.stringList(),
                    traffic_signals_slm_.stringList(),
                    rsu_object_topics_,
                    rsu_freespace_topics_,
                    rsu_signal_topics_,
                    obu_object_topics_,
                    obu_freespace_topics_,
                    obu_signal_topics_,
                    obu_tf_topics_,
                    link_topics_);

        std::set<std::string> crp_ids;
        common::getUniqueCrpIds(
                    traffic_signals_slm_.stringList(),
                    crp_ids);

        common::updateGeneralTopicsRviz(
                    rsu_ids,
                    obu_ids,
                    cloud_ids,
                    crp_ids,
                    rsu_object_topics_,
                    rsu_freespace_topics_,
                    obu_object_topics_,
                    obu_freespace_topics_);

        rsu_slm_.setStringList(rsu_ids);
        obu_slm_.setStringList(obu_ids);

        ui->VisualiserLaunchBagFile->setText(file_info.completeBaseName() + "." + file_info.completeSuffix());
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
    ui->labelVisualiserNetworkOffline1->setEnabled(arg1);
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
                    obu_object_topics_,
                    obu_freespace_topics_,
                    obu_signal_topics_,
                    ui->HeatmapRealTimeCheck->isChecked());

        common::updateNetworkTargetedLaunch(
                    std::to_string(std::stod(ui->NetworkRsuObuConDist->text().toStdString()))
                    , ui->NetworkSelectedRsu->currentText().toStdString()
                    , ui->NetworkSelectedObu->currentText().toStdString()
                    , ui->HeatmapOfflinePath->text().toStdString()
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
        ui->RosbagSuccess->clear();
        ui->RosbagError->setText(e.what());
    }
    catch (std::invalid_argument&) {
        ui->RosbagSuccess->clear();
        ui->RosbagError->setText("Error: Enter a double value for 'playback speed'");
    }
    catch (std::out_of_range&) {
        ui->RosbagSuccess->clear();
        ui->RosbagError->setText("Error: Invalid range for 'playback speed'");
    }
}


void MainWindow::on_VisualiserLaunch_clicked()
{
    try {
        if (ui->VisualiserLaunch->text() == "Launch") {
            if (main_app_process_.state() == QProcess::Running)
                throw std::runtime_error("The application is already running");

            QStringList arguments;
            arguments << "launch" << "visually_dm" << "visualiser.launch.py";
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

        arguments << (std::string(std::getenv("AVVV_DM_HOME")) + std::string("analyser/main.py")).c_str() << "plot";

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


void MainWindow::on_PreprocessorPreprocess_clicked()
{

    try {
        if (ui->PreprocessorPreprocess->text() == "Preprocess") {

            if (preprocessor_process_.state() == QProcess::Running)
                throw std::runtime_error("The application is already running");

            QString obu_tf_path{ ui->ObuTfPath->text() };
            QString obu_freespace_path{ ui->ObuFreespacePath->text() };
            QString obu_signal_path{ ui->ObuSignalPath->text() };
            QString obu_object_path{ ui->ObuObjectPath->text() };
            QString output_folder_path{ ui->PreprocessOutputPath->text() };
            QString obu_id{ ui->PreprocessObuId->text() };
            if (
                    obu_tf_path == "" or
                    obu_freespace_path == "" or
                    obu_signal_path == "" or
                    obu_object_path == "" or
                    output_folder_path == "" or
                    obu_id == "")
                throw std::runtime_error("Fill in all fields above");

            common::configurePreprocessConfig(
                        obu_tf_path,
                        obu_freespace_path,
                        obu_signal_path,
                        obu_object_path,
                        output_folder_path,
                        obu_id);

            QStringList arguments;

            arguments << "run" << "csv2bag" << "main";
            preprocessor_process_.start("ros2", arguments);
        }
        else {
            if (preprocessor_process_.state() == QProcess::Running)
                preprocessor_process_.kill();
        }
    }
    catch (std::runtime_error& e) {
        ui->PreprocessorError->setText(QString::fromStdString("Error: " + std::string(e.what())));
        ui->PreprocessorSuccess->clear();
    }


}


void MainWindow::on_ObuTfBrowse_clicked()
{
    QString path{ QFileDialog::getOpenFileName(this, tr("Open CSV files"), ".", tr("CSV Files (*.csv)")) };
    ui->ObuTfPath->setText(path);
}


void MainWindow::on_ObuFreespaceBrowse_clicked()
{
    QString path{ QFileDialog::getOpenFileName(this, tr("Open CSV files"), ".", tr("CSV Files (*.csv)")) };
    ui->ObuFreespacePath->setText(path);
}


void MainWindow::on_ObuSignalBrowse_clicked()
{
    QString path{ QFileDialog::getOpenFileName(this, tr("Open CSV files"), ".", tr("CSV Files (*.csv)")) };
    ui->ObuSignalPath->setText(path);
}


void MainWindow::on_ObuObjectBrowse_clicked()
{
    QString path{ QFileDialog::getOpenFileName(this, tr("Open CSV files"), ".", tr("CSV Files (*.csv)")) };
    ui->ObuObjectPath->setText(path);
}


void MainWindow::on_PreprocessOutputBrowse_clicked()
{
    QString path{ QFileDialog::getExistingDirectory(this, tr("Select a folder"), ".", QFileDialog::ShowDirsOnly) };
    ui->PreprocessOutputPath->setText(path);
}


void MainWindow::preprocessor_process_started()
{
    ui->PreprocessorError->clear();
    ui->PreprocessorSuccess->setText("Started generating the initial ROSBAG file...");
    ui->PreprocessorPreprocess->setText("Abort");
}


void MainWindow::preprocessor_process_error()
{
    ui->PreprocessorSuccess->clear();
    ui->PreprocessorError->setText("Error: Preprocessor failed. Make sure you've provided the correct CSVs");
    ui->PreprocessorPreprocess->setText("Preprocess");
    QApplication::beep();
}


void MainWindow::preprocessor_process_finished()

{
    ui->PreprocessorError->clear();
    ui->PreprocessorSuccess->setText("Done.");
    QString initial_bag_file_path{
        ui->PreprocessOutputPath->text()
            .append("/")
            .append(ui->PreprocessObuId->text()) };

    ui->AnalyserRosbagPath->setText(initial_bag_file_path);
    ui->PreprocessorPreprocess->setText("Preprocess");
    QApplication::beep();
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

    // @todo Update the paths specified in other tabs
    // referring to the generated ROSBAG folder.
    ui->VisualiserRosbagPath->setText(ui->AnalyserOutputPath->text().append("/outputs/rosbag2_avvv/rosbag2_avvv.db3"));
    ui->ReporterDataPath->setText(ui->AnalyserOutputPath->text().append("/outputs/graphs/"));
    ui->AnalyserAnalyse->setText("Analyse");
}


void MainWindow::rosbag_process_started()
{
    ui->RosbagError->clear();
    ui->RosbagSuccess->setText("Started playing the ROSBAG file");
    ui->RosbagPlay->setText("Stop");
}


void MainWindow::rosbag_process_error()
{
    ui->RosbagSuccess->clear();
    ui->RosbagError->setText("Error: Could not play ROSBAG, source the workspace");
    ui->RosbagPlay->setText("Play");
}


void MainWindow::rosbag_process_finished()
{
    ui->RosbagError->clear();
    ui->RosbagSuccess->setText("Finished playing the ROSBAG file");
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
