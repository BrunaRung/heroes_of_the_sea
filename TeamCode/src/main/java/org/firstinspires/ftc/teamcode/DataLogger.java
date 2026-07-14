package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * CSV data logger, copied from the ThunderStrike 33535 "Fortunate Son" code (rbx.util.DataLogger).
 *
 * <p>Writes rows to a fast internal temp file during the run, then on {@link #close()} copies the
 * file to the public Downloads directory as {@code <subsystemName>_<timestamp>.csv} (only if at
 * least one data row was written). Every row is prefixed with a {@code Time_Sec} column measured
 * from a shared epoch so rows from different logs line up in time. Two write APIs are provided:
 * the simple {@link #writeData(Object...)} and the zero-allocation fluent {@link #row()} / add /
 * {@link #end()} form for hot loops.
 */
public class DataLogger {

    private BufferedWriter writer;
    public final String subsystemName;
    /** Shared Time_Sec epoch across ALL DataLogger instances so rows from
     *  different logs are directly comparable. Defaults to class-load; reset
     *  once at run start via {@link #resetEpoch()}. */
    private static long sharedStartNanos = System.nanoTime();
    public double logWriteTime = 0;
    private boolean hasDataRows = false; // TRUE only if writeData() writes at least 1 row

    // We keep track of both files
    private File tempFile;         // Fast internal storage
    private String finalFileName;  // Name only, to avoid pre-creation

    /** Reused across rows so the per-row path allocates no StringBuilder.
     *  Single-threaded use: one logger writes one row at a time. */
    private final StringBuilder rowBuilder = new StringBuilder(2048);
    /** Reused char buffer so writeData() flushes without a line String. */
    private char[] rowChars = new char[2048];

    public DataLogger(String subsystemName) {
        this.subsystemName = subsystemName;

        try {
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            this.finalFileName = subsystemName + "_" + timestamp + ".csv";

            // Open the writer ONLY on the internal fast cache
            String tempDirPath = System.getProperty("java.io.tmpdir");
            this.tempFile = new File(tempDirPath, finalFileName);

            writer = new BufferedWriter(new FileWriter(tempFile));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /** Reset the shared {@code Time_Sec} epoch. Call once at OpMode start so
     *  every log written during the run shares the play-start origin and rows
     *  across logs are directly comparable. */
    public static void resetEpoch() {
        sharedStartNanos = System.nanoTime();
    }


    public void writeHeader(String header) {
        if (writer == null) return;
        try {
            writer.write("Time_Sec," + header);
            writer.newLine();
            writer.flush(); // Optional: Ensures header is saved even if crash occurs
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void writeData(Object... data) {
        logWriteTime = (double) System.nanoTime();

        if (writer == null) return;

        rowBuilder.setLength(0);
        appendTimeSec(rowBuilder, (System.nanoTime() - sharedStartNanos) / 1.0e9);
        for (int i = 0; i < data.length; i++) {
            rowBuilder.append(',').append(data[i]);
        }
        writeRow();

        logWriteTime = (double) System.nanoTime() - logWriteTime;
    }

    /** Fixed 4-decimal append, matching the prior
     *  String.format("%.4f", timeSec) without the Formatter allocation.
     *  Time_Sec is monotonic from the shared epoch, so v >= 0. */
    private static void appendTimeSec(StringBuilder sb, double v) {
        long scaled = Math.round(v * 10000.0);
        sb.append(scaled / 10000).append('.');
        long frac = scaled % 10000;
        if (frac < 1000) sb.append('0');
        if (frac < 100)  sb.append('0');
        if (frac < 10)   sb.append('0');
        sb.append(frac);
    }

    /** Flush rowBuilder to the writer using a reused char[] (no line String). */
    private void writeRow() {
        if (writer == null) return;
        try {
            int len = rowBuilder.length();
            if (rowChars.length < len) rowChars = new char[len];
            rowBuilder.getChars(0, len, rowChars, 0);
            writer.write(rowChars, 0, len);
            writer.newLine();
            hasDataRows = true;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /* ================================================================
     * Fluent zero-allocation row API (Lever 2). Same output as
     * writeData(Object...), but the typed add(...) overloads append
     * primitives straight into the reused rowBuilder — no varargs array,
     * no autoboxing, no per-field toString. The legacy writeData(Object...)
     * above is unchanged and stays the path for every existing caller;
     * only the hot per-tick chase loggers use this fluent form.
     * ================================================================ */

    /** Begin a row: reset the buffer and write the Time_Sec column. Chain
     *  typed add(...) calls, then end(). */
    public DataLogger row() {
        logWriteTime = (double) System.nanoTime();
        rowBuilder.setLength(0);
        appendTimeSec(rowBuilder, (System.nanoTime() - sharedStartNanos) / 1.0e9);
        return this;
    }

    public DataLogger add(double v)  { rowBuilder.append(',').append(v); return this; }
    public DataLogger add(long v)    { rowBuilder.append(',').append(v); return this; }
    public DataLogger add(int v)     { rowBuilder.append(',').append(v); return this; }
    public DataLogger add(boolean v) { rowBuilder.append(',').append(v); return this; }
    public DataLogger add(String v)  { rowBuilder.append(',').append(v); return this; }
    /** Enums / other objects (e.g. abort-reason). An enum's toString is its
     *  stored name — no allocation. Separate from add(String) only so call
     *  sites can pass an enum directly. */
    public DataLogger add(Object v)  { rowBuilder.append(',').append(v); return this; }

    /** Terminate and flush the row built by row()/add(...). */
    public void end() {
        writeRow();
        logWriteTime = (double) System.nanoTime() - logWriteTime;
    }

    // Close the file and MOVE it to SD card
    public void close() {
        try {
            if (writer != null) {
                writer.flush();
                writer.close();
                writer = null;
            }

            if (tempFile != null && tempFile.exists()) {
                // If we never wrote any real data rows, delete temp and DO NOT copy
                if (!hasDataRows) {
                    tempFile.delete();
                    return;
                }

                // Construct the destination path ONLY NOW
                File publicPath = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
                if (!publicPath.exists()) publicPath.mkdirs();

                File finalFile = new File(publicPath, finalFileName);

                // Manual copy
                copyFileManual(tempFile, finalFile);

                // Delete the internal temp file
                tempFile.delete();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    private void copyFileManual(File source, File dest) throws IOException {
        java.io.InputStream in = new java.io.FileInputStream(source);
        java.io.OutputStream out = new java.io.FileOutputStream(dest);

        byte[] buffer = new byte[1024];
        int length;
        while ((length = in.read(buffer)) > 0) {
            out.write(buffer, 0, length);
        }
        in.close();
        out.close();
    }
}
