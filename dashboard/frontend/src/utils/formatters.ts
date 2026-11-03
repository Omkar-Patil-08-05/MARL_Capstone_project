/**
 * Unified Formatting Utility for Antigravity Dashboard
 * Search and Rescue Multi-Agent Swarm
 */

/**
 * Distinguishes fractional coverage values (e.g. 0.30504, 0.9008)
 * from already-scaled percentage values (e.g. 30.5, 90.08) and formats them cleanly.
 *
 * Rules:
 * - 0.30504 -> "30.50%" (digits=2) or "30.5%" (digits=1)
 * - 0.9008 -> "90.08%"
 * - 30.5 -> "30.50%"
 * - 1.0 -> "100.00%" (fractional 1.0 represents 100%)
 * - 0 -> "0.00%"
 * - null/undefined/NaN -> "0.00%"
 */
export function formatCoverage(value: number | string | null | undefined, digits: number = 2): string {
    if (value === null || value === undefined || value === '') {
        return `0.${'0'.repeat(digits)}%`;
    }
    const num = typeof value === 'number' ? value : parseFloat(value);
    if (isNaN(num) || num <= 0) {
        return `0.${'0'.repeat(digits)}%`;
    }
    // Unit fractions (0 < x <= 1.0) scaled to percentage (e.g. 0.9008 -> 90.08, 0.30504 -> 30.50)
    const pct = num <= 1.0 ? num * 100 : num;
    return `${pct.toFixed(digits)}%`;
}

/**
 * Returns numeric percentage in range [0, 100] suitable for SVG coordinates, progress bars, and graph plotting.
 */
export function getCoveragePercent(value: number | string | null | undefined): number {
    if (value === null || value === undefined || value === '') {
        return 0;
    }
    const num = typeof value === 'number' ? value : parseFloat(value);
    if (isNaN(num) || num <= 0) {
        return 0;
    }
    const pct = num <= 1.0 ? num * 100 : num;
    return Math.min(100, Math.max(0, pct));
}

/**
 * Safe number formatter that handles null/undefined/NaN cleanly with a fallback.
 */
export function formatNumber(value: number | null | undefined, digits: number = 1, fallback: string = '0'): string {
    if (value === null || value === undefined || isNaN(value)) {
        return fallback;
    }
    return value.toFixed(digits);
}
