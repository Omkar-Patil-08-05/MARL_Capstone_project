import puppeteer from 'puppeteer';
import http from 'http';

function post(url, data) {
    return new Promise((resolve, reject) => {
        const u = new URL(url);
        const req = http.request({
            hostname: u.hostname,
            port: u.port,
            path: u.pathname,
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Content-Length': Buffer.byteLength(JSON.stringify(data))
            }
        }, (res) => {
            let body = '';
            res.on('data', chunk => body += chunk);
            res.on('end', () => resolve({ status: res.statusCode, body: JSON.parse(body || '{}') }));
        });
        req.on('error', reject);
        req.write(JSON.stringify(data));
        req.end();
    });
}

function get(url) {
    return new Promise((resolve, reject) => {
        http.get(url, (res) => {
            let body = '';
            res.on('data', chunk => body += chunk);
            res.on('end', () => resolve({ status: res.statusCode, body: JSON.parse(body || '{}') }));
        }).on('error', reject);
    });
}

(async () => {
    console.log("=================================================");
    console.log("STARTING LIVE BROWSER CAMERA STREAM UI VERIFICATION");
    console.log("=================================================");

    const browser = await puppeteer.launch({
        executablePath: '/usr/bin/google-chrome',
        args: ['--no-sandbox', '--disable-setuid-sandbox', '--disable-gpu', '--window-size=1920,1080']
    });

    try {
        const page = await browser.newPage();
        await page.setViewport({ width: 1920, height: 1080 });

        console.log("[PUPPETEER] Navigating to http://localhost:5173...");
        await page.goto('http://localhost:5173', { waitUntil: 'networkidle0' });

        console.log("[MISSION] Starting realistic_sar mission via backend API...");
        const startRes = await post('http://localhost:8000/api/mission/start', {
            map_id: 'realistic_sar',
            drone_count: 2,
            victim_count: 3
        });
        console.log("[MISSION] Start response:", startRes);

        // Wait for the active mission view to appear
        console.log("[UI] Waiting for active mission dashboard and drone cards...");
        await page.waitForFunction(() => {
            // Check if Drone cards are rendered
            const text = document.body.innerText;
            return text.includes('DRONE 0') || text.includes('DRONE_0');
        }, { timeout: 120000, polling: 1000 });

        console.log("[UI] Drone cards detected on page!");

        // Wait a few seconds for telemetry and video pipeline
        await new Promise(r => setTimeout(r, 5000));

        // Find CAM toggle button and click it
        console.log("[UI] Clicking CAM toggle button on Drone card...");
        const camButtons = await page.$$('button[title*="Camera"], button[title*="CAM"], button');
        let clicked = false;
        for (const btn of camButtons) {
            const btnText = await page.evaluate(el => el.textContent, btn);
            if (btnText && btnText.includes('CAM')) {
                await btn.click();
                clicked = true;
                console.log("[UI] Clicked CAM toggle button!");
                break;
            }
        }
        if (!clicked) {
            console.log("[WARNING] Could not find specific CAM button, looking for any button inside Drone card");
        }

        // Wait for camera container or image to render
        await new Promise(r => setTimeout(r, 4000));

        // Inspect camera image elements
        const imgMetrics = await page.evaluate(() => {
            const imgs = Array.from(document.querySelectorAll('img[alt*="Camera Stream"], img[src*="/api/camera/stream"]'));
            const errorTextPresent = document.body.innerText.includes('CAMERA SIGNAL CONNECTING') || document.body.innerText.includes('RETRY FEED');
            return {
                imgCount: imgs.length,
                images: imgs.map(img => ({
                    src: img.src,
                    complete: img.complete,
                    naturalWidth: img.naturalWidth,
                    naturalHeight: img.naturalHeight,
                    displayedWidth: img.clientWidth,
                    displayedHeight: img.clientHeight
                })),
                hasErrorText: errorTextPresent
            };
        });

        console.log("[UI CAMERA METRICS]", JSON.stringify(imgMetrics, null, 2));

        const screenshotPath = '/tmp/dashboard_camera_stream_verified.png';
        await page.screenshot({ path: screenshotPath });
        console.log(`[PUPPETEER] Screenshot saved to ${screenshotPath}`);

        // Stop the mission cleanly
        console.log("[MISSION] Stopping mission...");
        await post('http://localhost:8000/api/mission/stop', {});
        console.log("[MISSION] Mission stopped.");

        // Assertions
        if (imgMetrics.imgCount === 0) {
            throw new Error("No camera stream <img> elements found in DOM!");
        }
        for (const img of imgMetrics.images) {
            if (img.naturalWidth === 0 || img.naturalHeight === 0) {
                throw new Error(`Camera image failed to decode in browser: ${JSON.stringify(img)}`);
            }
        }
        if (imgMetrics.hasErrorText) {
            throw new Error("UI still displays 'CAMERA SIGNAL CONNECTING' or 'RETRY FEED'!");
        }

        console.log("=================================================");
        console.log("LIVE BROWSER CAMERA VERIFICATION PASSED 100%!");
        console.log("=================================================");
    } finally {
        await browser.close();
    }
})();
