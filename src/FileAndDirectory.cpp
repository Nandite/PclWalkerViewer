#include <QFileDialog>
#include "FileAndDirectory.hpp"

std::string io::selectDirectoryFromDialog(const std::string &title) {
    QFileDialog fileSelectionDialog(nullptr);
    fileSelectionDialog.setFileMode(QFileDialog::DirectoryOnly);
    fileSelectionDialog.setViewMode(QFileDialog::ViewMode::Detail);
    fileSelectionDialog.setWindowTitle(title.c_str());
    fileSelectionDialog.setOption(QFileDialog::DontResolveSymlinks);
    fileSelectionDialog.setOption(QFileDialog::ShowDirsOnly);
    fileSelectionDialog.setDirectory(QDir::homePath());
    if (fileSelectionDialog.exec()) {
        fileSelectionDialog.close();
        const auto selectedDirectories{fileSelectionDialog.selectedFiles()};
        return selectedDirectories.back().toStdString();
    }
    return {};
}
