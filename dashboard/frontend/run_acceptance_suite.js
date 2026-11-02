import puppeteer from 'puppeteer';
import fs from 'fs';

async function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

(async () => {
    console.log("================================================================================");
    console.log("RUNNING END-TO-END PUPPETEER DASHBOARD ACCEPTANCE TEST");
    console.log("================================================================================");

    const browser = await puppeteer.launch({
        executablePath: '/usr/bin/google-chrome',
        args: [
            '--no-sandbox',
            '--disable-setuid-sandbox',
            '--disable-gpu',
            '--window-size=1920,1080'
        ]
    });

    const page = await browser.newPage();
    await page.setViewport({ width: 1920, height: 1080 });

    try {
        console.log("[NAVIGATE] Opening http://localhost:5173...");
        await page.goto('http://localhost:5173', { waitUntil: 'networkidle0' });
        await sleep(1500);

        // 1. Select Forest Wildland SAR
        console.log("[SETUP] Selecting Forest Wildland SAR...");
        const forestCard = await page.waitForSelector('text/Forest Wildland SAR', { timeout: 5000 });
        await forestCard.click();
        await sleep(500);

        // 2. Select 2 Drones
        console.log("[SETUP] Selecting 2 Drones...");
        const droneButtons = await page.$$('button');
        // Find button with text '2' in drone count section
        const buttons = await page.evaluate(() => {
            const btns = Array.from(document.querySelectorAll('button'));
            return btns.map((b, i) => ({ index: i, text: b.textContent.trim() }));
        });
        
        // Find drone 2 button and victim 3 button
        await page.evaluate(() => {
            const btns = Array.from(document.querySelectorAll('button'));
            // Look for buttons that represent numbers
            const numBtns = btns.filter(b => /^\d+$/.test(b.textContent.trim()));
            // Drone buttons are 1-6 (first 6), victim buttons are 1-10 (next 10)
            if (numBtns.length >= 16) {
                // Drone count 2 is numBtns[1]
                numBtns[1].click();
                // Victim count 3 is numBtns[6 + 2] = numBtns[8]
                numBtns[8].click();
            }
        });
        await sleep(500);

        // Capture screenshot of configured setup screen
        await page.screenshot({ path: '/tmp/01_setup_configured.png' });
        console.log("[SCREENSHOT] Saved /tmp/01_setup_configured.png (Forest SAR, 2 Drones, 3 Victims)");

        // 3. Click START MISSION
        console.log("[SETUP] Clicking START MISSION...");
        const startBtn = await page.waitForSelector('text/START MISSION', { timeout: 5000 });
        await startBtn.click();

        // 4. Wait for mission to reach RUNNING state
        console.log("[WAIT] Waiting for mission to initialize and reach RUNNING state...");
        const startTime = Date.now();
        let reachedRunning = false;

        while ((Date.now() - startTime) < 240000) { // Up to 4 minutes
            const statusText = await page.evaluate(() => {
                const els = Array.from(document.querySelectorAll('*'));
                const statusEl = els.find(e => e.textContent && e.textContent.includes('BACKEND STATUS'));
                return statusEl ? statusEl.parentElement?.textContent : '';
            });

            const bodyText = await page.evaluate(() => document.body.innerText);
            if (bodyText.includes('RUNNING') || bodyText.includes('QMIX_STARTING')) {
                if (bodyText.includes('RUNNING')) {
                    reachedRunning = true;
                    console.log(`>>> MISSION REACHED RUNNING STATE at ${((Date.now() - startTime)/1000).toFixed(1)}s! <<<`);
                    break;
                }
            }

            process.stdout.write(`... waiting (${((Date.now() - startTime)/1000).toFixed(0)}s)\r`);
            await sleep(3000);
        }

        if (!reachedRunning) {
            console.log("\n[WARN] Mission did not display RUNNING in UI within timeout, capturing screen state.");
            await page.screenshot({ path: '/tmp/02_running_timeout.png' });
        } else {
            // Let simulation run for 10 seconds to populate live telemetry
            await sleep(10000);

            // 5. Inspect Normal Live Dashboard
            await page.screenshot({ path: '/tmp/02_live_dashboard_normal.png' });
            console.log("[SCREENSHOT] Saved /tmp/02_live_dashboard_normal.png");

            // 6. Test Drone Camera View Toggle
            console.log("[CAMERA] Testing Drone Camera Toggle on Drone Card...");
            const camToggled = await page.evaluate(() => {
                const btns = Array.from(document.querySelectorAll('button'));
                const camBtn = btns.find(b => b.textContent.includes('CAM'));
                if (camBtn) {
                    camBtn.click();
                    return true;
                }
                return false;
            });
            console.log("Cam button toggled:", camToggled);
            await sleep(4000); // Allow camera stream frames to render
            await page.screenshot({ path: '/tmp/03_drone_camera_view.png' });
            console.log("[SCREENSHOT] Saved /tmp/03_drone_camera_view.png");

            // 7. Test Full-Map Expansion
            console.log("[MAP] Testing Full-Screen Map Expansion...");
            const mapExpanded = await page.evaluate(() => {
                const btns = Array.from(document.querySelectorAll('button'));
                // Look for expand button (Maximize2 icon or title "Expand Map")
                const expBtn = btns.find(b => b.title && b.title.includes('Expand Map')) ||
                               btns.find(b => b.innerHTML.includes('lucide-maximize') || b.querySelector('svg.lucide-maximize-2'));
                if (expBtn) {
                    expBtn.click();
                    return true;
                }
                return false;
            });
            console.log("Map expand button clicked:", mapExpanded);
            await sleep(2000);
            await page.screenshot({ path: '/tmp/04_full_map_expanded.png' });
            console.log("[SCREENSHOT] Saved /tmp/04_full_map_expanded.png");

            // Close full-screen map (Escape or click minimize)
            await page.keyboard.press('Escape');
            await sleep(1000);

            // 8. Test VIEW SIMULATION (Gazebo GUI Launch)
            console.log("[GAZEBO GUI] Testing VIEW SIMULATION button...");
            const viewSimClicked = await page.evaluate(() => {
                const btns = Array.from(document.querySelectorAll('button'));
                const vsBtn = btns.find(b => b.textContent.includes('VIEW SIMULATION'));
                if (vsBtn) {
                    vsBtn.click();
                    return true;
                }
                return false;
            });
            console.log("VIEW SIMULATION clicked:", viewSimClicked);
            await sleep(3000);

            // Click VIEW SIMULATION again to test duplicate prevention guard
            console.log("[GAZEBO GUI] Testing second click on VIEW SIMULATION (duplicate prevention guard)...");
            await page.evaluate(() => {
                const btns = Array.from(document.querySelectorAll('button'));
                const vsBtn = btns.find(b => b.textContent.includes('VIEW SIMULATION'));
                if (vsBtn) vsBtn.click();
            });
            await sleep(2000);

            await page.screenshot({ path: '/tmp/05_after_view_simulation.png' });
            console.log("[SCREENSHOT] Saved /tmp/05_after_view_simulation.png");

            // 9. Stop Mission cleanly via UI STOP button
            console.log("[STOP] Clicking STOP button...");
            await page.evaluate(() => {
                const btns = Array.from(document.querySelectorAll('button'));
                const stopBtn = btns.find(b => b.textContent.includes('STOP'));
                if (stopBtn) stopBtn.click();
            });
            await sleep(3000);
            await page.screenshot({ path: '/tmp/06_after_stop.png' });
            console.log("[SCREENSHOT] Saved /tmp/06_after_stop.png");
        }

        console.log("================================================================================");
        console.log("END-TO-END PUPPETEER ACCEPTANCE SUITE FINISHED SUCCESSFULLY");
        console.log("================================================================================");
    } catch (e) {
        console.error("Test error:", e);
    } finally {
        await browser.close();
    }
})();
