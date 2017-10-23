# HTC Vive on Ubuntu 16.04

Instructions as of 18/10/17.

First install Ubuntu 16.04(.03 right now).

Follow instructions of [SteamVR instructions](https://github.com/ValveSoftware/SteamVR-for-Linux).

```bash
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt-get update
sudo apt-get install nvidia-384 nvidia-384-dev
```

Reboot. Hope to not have destroyed your X.

Install Steam (about 1GB).

```bash
sudo apt-get install steam
```

Open Steam, wait for it to update, login.

Go to `Steam > Settings` and add Beta participation. It will reboot Steam.

Now we want to install SteamVR, but in beta mode, so look for SteamVR somewhere in the client (for me it's in `Library` already, you may find it by clicking [this link](steam://run/250820)) and right click
Properties, BETA tab, and add yourself to the beta. Now install. (It's like 1GB).

Execute SteamVR and do the `room setup`.

