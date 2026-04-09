const {app, BrowserWindow} = require('electron');

function createWindow() {
  const win = new BrowserWindow({
    width: 800,
    height: 600,
    autoHideMenuBar: true,
    webPreferences: {
        enableRemoteModule: true,
    },
  });

  win.loadURL('http://127.0.0.1:3000');

  win.webContents.on('did-finish-load', () => {
    win.webContents.setZoomLevel(-2);
  });
}

app.whenReady().then(createWindow);

