#include <windows.h>
#include <stdio.h>

void main()
{
    STARTUPINFO si;
    PROCESS_INFORMATION pi;

    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));

    LPSTR cmd = const_cast<char*>(".\\SkySocketPython\\python.exe __main__.py");
    // Start the child process. 
    if (!CreateProcess(NULL,   // No module name (use command line)
        cmd,
        NULL,           // Process handle not inheritable
        NULL,           // Thread handle not inheritable
        FALSE,          // Set handle inheritance to FALSE
        CREATE_NO_WINDOW,              // No creation flags
        NULL,           // Use parent's environment block
        NULL,           // Use parent's starting directory 
        &si,            // Pointer to STARTUPINFO structure
        &pi)           // Pointer to PROCESS_INFORMATION structure
        )
    {
        printf("SkySocketLaunch failed (%d).\n", GetLastError());
        return;
    }
}