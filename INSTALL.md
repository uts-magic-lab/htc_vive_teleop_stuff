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

Install Steam (about 1GB) and a few dependences.

```bash
sudo apt-get install steam libsdl2-dev libvulkan-dev libudev-dev libssl-dev zlib1g-dev python-pip
```

Make a Symbolic Link from libudev.so.0 to libudev.so.1 for SteamVR to use.

```bash
sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1 /lib/x86_64-linux-gnu/libudev.so.0
```

Open Steam, wait for it to update, login.

Go to `Steam > Settings` and add Beta participation. It will reboot Steam.

Now we want to install SteamVR, but in beta mode, so look for SteamVR somewhere in the client (for me it's in `Library` already, you may find it by going to [this link](https://steamdb.info/app/250820/) and clicking the green icon with the '(cloud) Free') and right click
Properties, BETA tab, and add yourself to the beta. Now install. (It's like 1GB).

Execute SteamVR and do the `room setup`.

