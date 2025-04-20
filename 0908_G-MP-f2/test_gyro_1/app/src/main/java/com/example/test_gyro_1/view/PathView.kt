package com.example.test_gyro_1.view

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.PointF //PointFを使用
import android.util.AttributeSet
import android.view.View
import kotlin.math.max
import kotlin.math.min

class PathView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null, defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private var currentPoint: PointF? = null //現在位置を保持する変数
    private val path = Path() //描画用のPathオブジェクト
    private val paint = Paint().apply {
        color = Color.BLUE //線の色
        strokeWidth = 5f     //線の太さ
        style = Paint.Style.STROKE //線で描画
        isAntiAlias = true    //アンチエイリアス有効
    }

    //現在位置用のPaint
    private val currentPointPaint = Paint().apply {
        color = Color.GREEN
        style = Paint.Style.FILL // 塗りつぶし
        isAntiAlias = true
    }
    private val originPointPaint = Paint().apply {
        color = Color.RED
        style = Paint.Style.FILL
        isAntiAlias = true
    }

    //描画範囲の計算用
    private var minX = 0f
    private var maxX = 0f
    private var minY = 0f
    private var maxY = 0f
    private var scale = 5f //座標スケール (1メートルを何ピクセルで描画するか) - 要調整
    private var offsetX = 0f //X方向のオフセット (View中央に原点が来るように)
    private var offsetY = 0f //Y方向のオフセット (View中央に原点が来るように)

    //データ更新用メソッド
    fun updatePath(newPoints: List<PointF>, currentPoint: PointF?) {
        //リストが空でなければ描画範囲とスケールを再計算
        val pointsToConsider = newPoints + listOfNotNull(currentPoint) // 現在位置も範囲計算に含める
        if (pointsToConsider.isNotEmpty()) {
            calculateBoundsAndScale(pointsToConsider)
        } else {
            //データが空なら範囲をリセット
            minX = 0f; maxX = 0f; minY = 0f; maxY = 0f;
        }
        this.currentPoint = currentPoint

        //座標変換してPathを再構築
        path.rewind() //Pathをリセット
        if (newPoints.isNotEmpty()) {
            val start = transformPoint(newPoints[0])
            path.moveTo(start.x, start.y)
            for (i in 1 until newPoints.size) {
                val point = transformPoint(newPoints[i])
                path.lineTo(point.x, point.y)
            }
        }

        invalidate() // 再描画を要求
    }

    //描画範囲とスケール、オフセットを計算
    private fun calculateBoundsAndScale(points: List<PointF>) {
        if (points.isEmpty()) return

        minX = points.minOf { it.x }
        maxX = points.maxOf { it.x }
        minY = points.minOf { it.y }
        maxY = points.maxOf { it.y }

        //Viewのサイズ取得 (onSizeChangedで取得するのがより正確だが、ここでは簡易的に)
        val viewWidth = width.toFloat()
        val viewHeight = height.toFloat()
        if (viewWidth == 0f || viewHeight == 0f) return // サイズ未確定なら計算しない

        //必要な描画範囲（メートル単位）
        val requiredWidthMeters = max(1f, maxX - minX) // 最低1m幅は確保
        val requiredHeightMeters = max(1f, maxY - minY)

        //Viewのサイズに合わせてスケールを決定 (パディングも考慮)
        val padding = 30f * 2 // 左右・上下のパディング合計 (ピクセル)
        val availableWidth = viewWidth - padding
        val availableHeight = viewHeight - padding

        val scaleX = if (requiredWidthMeters > 0) availableWidth / requiredWidthMeters else 10f
        val scaleY = if (requiredHeightMeters > 0) availableHeight / requiredHeightMeters else 10f

        //X, Yで小さい方のスケールを採用し、アスペクト比を保つ
        scale = min(scaleX, scaleY)
        if (scale <= 0) scale = 1f // スケールが0以下になるのを防ぐ

        //軌跡の中心がViewの中心に来るようにオフセットを計算
        val centerX = (minX + maxX) / 2f
        val centerY = (minY + maxY) / 2f
        offsetX = viewWidth / 2f - centerX * scale
        offsetY = viewHeight / 2f + centerY * scale // Y軸反転考慮
    }

    //EKF座標をView座標に変換
    private fun transformPoint(point: PointF): PointF {
        //スケール適用とY軸反転、オフセット適用
        val viewX = point.x * scale + offsetX
        val viewY = -point.y * scale + offsetY // Y軸反転
        return PointF(viewX, viewY)
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        //Pathを描画
        canvas.drawPath(path, paint)

        //オプション: 原点(0,0)や現在の位置に目印を描画
        val origin = transformPoint(PointF(0f, 0f))
        canvas.drawCircle(origin.x, origin.y, 10f, originPointPaint) // 原点に赤丸
        currentPoint?.let {
            val currentViewPoint = transformPoint(it)
            canvas.drawCircle(currentViewPoint.x, currentViewPoint.y, 8f, currentPointPaint) // 現在位置に緑丸
        }

        //pathPointsリストの最後の点を描画する場合 (保持している場合)
        //if (pathPoints.isNotEmpty()) {
        //   val lastPoint = transformPoint(pathPoints.last())
        //   canvas.drawCircle(lastPoint.x, lastPoint.y, 8f, Paint().apply { color = Color.GREEN }) // 現在位置に緑丸
        //}
    }

    //Viewサイズ変更時に再計算 (スケール決定に使う)
    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        //サイズが変わったらスケールとオフセットを再計算する必要がある
        //calculateBoundsAndScale(pathPoints) //pathPointsをクラス変数として保持する場合
        //invalidate() //再描画
        //updatePath()を再度呼ぶ設計にするのがシンプルかも
    }
}